use crate::capture::driver::{CaptureConfig, CapturedFrameKind};
use crate::capture::{IMAGE_HEIGHT, IMAGE_WIDTH};
use crate::components::{
    Controlled, InfantryChassis, InfantryGimbal, InfantryLaunchOffset, SubscribeAutoAim,
};
use crate::config::SimulationConfig;

use crate::talos::capture::{
    TalosCaptureContext, TalosCapturePlugin, TalosFrameStamp, advance_talos_frame_stamp,
    publish_talos_runtime_state_system,
};

use bevy::image::BevyDefault;
use bevy::prelude::*;
use bevy::render::render_resource::TextureFormat;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use talos_ipc::*;

#[derive(Resource)]
pub struct ShmSubscriberRes(pub Arc<Mutex<ShmSubscriber>>);

#[derive(Resource, Deref, DerefMut)]
pub struct TalosEnabled(pub AtomicBool);

pub struct TalosPluginConfig {
    pub width: u32,
    pub height: u32,
    pub fov_y: f32,
    pub texture_format: TextureFormat,
}

impl Default for TalosPluginConfig {
    fn default() -> Self {
        let config = SimulationConfig::default();
        Self {
            width: IMAGE_WIDTH,
            height: IMAGE_HEIGHT,
            fov_y: config.camera.fov.to_radians(),
            texture_format: TextureFormat::Rgba8UnormSrgb,
        }
    }
}

#[derive(Default)]
pub struct TalosPlugin {
    pub config: TalosPluginConfig,
}

impl Plugin for TalosPlugin {
    fn build(&self, app: &mut App) {
        let publisher = match ShmPublisher::create() {
            Ok(p) => {
                info!("talos shm created");
                p
            }
            Err(e) => {
                error!("cannot create talos shm: {}", e);
                return;
            }
        };

        let publisher = Arc::new(Mutex::new(publisher));

        let capture_config = CaptureConfig {
            width: self.config.width,
            height: self.config.height,
            texture_format: self.config.texture_format,
            frame_kind: CapturedFrameKind::Rgb8,
        };

        let capture_context = TalosCaptureContext {
            publisher: publisher.clone(),
            fov_y: self.config.fov_y,
        };

        app.init_resource::<TalosFrameStamp>();

        app.add_plugins(TalosCapturePlugin {
            config: capture_config,
            context: capture_context,
        });

        match ShmSubscriber::connect() {
            Ok(subscriber) => {
                info!("connected to talos-cpp");
                app.insert_resource(ShmSubscriberRes(Arc::new(Mutex::new(subscriber))));
            }
            Err(_) => {
                info!("could not connect to talos-cpp");
            }
        }

        app.insert_resource(TalosEnabled(AtomicBool::new(true)));
        app.add_systems(Last, (advance_talos_frame_stamp, heartbeat_system));
        app.add_systems(
            Last,
            publish_talos_runtime_state_system.after(advance_talos_frame_stamp),
        );
        app.add_systems(
            Last,
            crate::talos::ground_truth::publish_ground_truth_system
                .after(publish_talos_runtime_state_system),
        );
        app.add_systems(
            Last,
            process_subscription
                .in_set(crate::operator::Execution::Receive)
                .run_if(|enabled: Res<SubscribeAutoAim>| enabled.load(Ordering::Acquire)),
        );
    }
}

fn process_subscription(
    mut operator: ResMut<crate::operator::OperatorState>,
    context: Option<Res<ShmSubscriberRes>>,

    config: Res<SimulationConfig>,
    gimbal: Single<
        (&mut Transform, &mut InfantryGimbal, &ChildOf),
        (
            With<Controlled>,
            Without<InfantryChassis>,
            Without<InfantryLaunchOffset>,
        ),
    >,
    muzzle_offset: Single<&Transform, (With<InfantryLaunchOffset>, With<Controlled>)>,
    parent_transforms: Query<&GlobalTransform, Without<InfantryGimbal>>,
) {
    let Some(ctx) = context else {
        return;
    };
    let (mut gimbal_transform, mut gimbal_data, gimbal_parent) = gimbal.into_inner();

    let Some(cmd) = recv_gimbal_cmd(&ctx) else {
        return;
    };
    if !cmd.distance_m.is_finite() || cmd.distance_m < 0.0 {
        return;
    }
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default().as_nanos() as u64;
    if !operator.accept_command(cmd.timestamp_ns, now) {
        return;
    }
    let Some((world_yaw, world_pitch)) = command_angles(cmd) else {
        warn!(
            "ignoring invalid Talos world-angle command: yaw={} pitch={}",
            cmd.yaw_deg, cmd.pitch_deg
        );
        return;
    };
    let Ok(parent_transform) = parent_transforms.get(gimbal_parent.0) else {
        warn!("ignoring Talos gimbal command: gimbal parent transform is unavailable");
        return;
    };
    let Some((local_rotation, local_yaw, local_pitch)) = world_command_to_local_rotation(
        world_yaw,
        world_pitch,
        parent_transform.rotation(),
        muzzle_offset.rotation,
        config.vehicle.gimbal_pitch_limit,
    ) else {
        warn!(
            "ignoring unreachable Talos world-angle command: yaw={} pitch={}",
            cmd.yaw_deg, cmd.pitch_deg
        );
        return;
    };
    gimbal_data.local_yaw = local_yaw;
    gimbal_data.pitch = local_pitch;
    gimbal_transform.rotation = local_rotation;
    operator.automatic_request = cmd.fire_advice == 1;
    operator.reason = if !operator.fire_authorized { "not_authorized" }
        else if cmd.fire_advice != 1 { "no_fire_advice" } else { "aim_command_applied" }.into();
    //info!("yaw={} pitch={}", cmd.yaw_deg, cmd.pitch_deg);
}

fn wrap_yaw(angle: f32) -> f32 {
    (angle + std::f32::consts::PI).rem_euclid(std::f32::consts::TAU) - std::f32::consts::PI
}

fn command_angles(cmd: GimbalCmd) -> Option<(f32, f32)> {
    if !cmd.yaw_deg.is_finite() || !cmd.pitch_deg.is_finite() {
        return None;
    }

    let world_pitch = cmd.pitch_deg.to_radians();
    if !(-std::f32::consts::FRAC_PI_2..=std::f32::consts::FRAC_PI_2).contains(&world_pitch) {
        return None;
    }

    Some((wrap_yaw(cmd.yaw_deg.to_radians()), world_pitch))
}
