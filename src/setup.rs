use avian3d::prelude::*;
use bevy::anti_alias::fxaa::Fxaa;
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::prelude::*;
use bevy::world_serialization::{WorldInstance, WorldInstanceReady};
use bevy_inspector_egui::bevy_egui::{EguiGlobalSettings, PrimaryEguiContext};
use std::collections::HashMap;

use crate::components::{
    ActiveSlapper, Controlled, DartLaunch, GameLayer, GroundRoot, Infantry, InfantryChassis,
    InfantryGimbal, InfantryLaunchOffset, InfantryViewOffset, MainCamera, PreciousCollision,
    SlapperInfantry,
};
use crate::config::SimulationConfig;
use crate::metalfx::MetalFxTemporalUpscaling;
use crate::robomaster::prelude::{
    HERO_ROBOT_CONFIG, INFANTRY_THREE_CONFIG, OutpostRoot, PowerRuneRoot, ScanArmor, Team,
    TechCoreRoot,
};
use crate::robomaster::vehicle::movement::VehicleDynamic;
use crate::systems::spawn_text;
use crate::util::entity_query::HierarchyQuery;

#[derive(Component)]
pub struct ScanOutpost;

const VEHICLE_SPAWN_HEIGHT: f32 = 0.50;
const VEHICLE_COLLIDER_Y_OFFSET: f32 = -0.115649;
const VEHICLE_COLLIDER_RADIUS: f32 = 0.2593615;
const CONTROLLED_VEHICLE_SPAWN: Vec3 = Vec3::new(0.0, VEHICLE_SPAWN_HEIGHT, 4.0);
const PRIMARY_TARGET_SPAWN: Vec3 = Vec3::new(0.7, VEHICLE_SPAWN_HEIGHT, 2.35);
const SECONDARY_TARGET_SPAWN: Vec3 = Vec3::new(2.0, VEHICLE_SPAWN_HEIGHT, 1.0);
const VEHICLE_RECOVERY_UP_DOT: f32 = 0.5;
const VEHICLE_RECOVERY_LIFT: f32 = 0.08;

pub fn setup(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    config: Res<SimulationConfig>,
    egui_global_settings: Option<ResMut<EguiGlobalSettings>>,
) {
    if let Some(mut egui_global_settings) = egui_global_settings {
        egui_global_settings.auto_create_primary_context = false;
    }
    spawn_text(&mut commands);
    commands.spawn((
        DirectionalLight {
            color: Color::srgb(0.9, 0.95, 1.0),
            illuminance: config.render.illuminance,
            shadow_maps_enabled: config.render.shadows,
            contact_shadows_enabled: config.render.shadows,
            ..default()
        },
        Transform::from_xyz(0.0, 4.0, 0.0).looking_at(Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)),
    ));

    let layer_env = GameLayer::environment_collision_layers();

    let trimesh = || {
        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
            TrimeshFlags::all(),
        ))
        .with_default_layers(layer_env)
    };
    let voxel = |size| {
        ColliderConstructorHierarchy::new(ColliderConstructor::VoxelizedTrimeshFromMesh {
            voxel_size: size,
            fill_mode: FillMode::FloodFill {
                detect_cavities: true,
            },
        })
        .with_default_layers(layer_env)
    };

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("GROUND.glb"))),
        Transform::IDENTITY,
        GroundRoot,
        Friction::new(0.5),
        PreciousCollision(HashMap::from([(
            "GROUND_DENSE".to_string(),
            (
                trimesh(),
                layer_env,
                Visibility::Visible,
                Some(RigidBody::Static),
            ),
        )])),
    ));

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("CALIB.glb"))),
        Transform::IDENTITY
            .with_scale(Vec3::splat(1.0))
            .with_translation(Vec3::new(1.0, 2.5, 1.0)),
    ));

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("CALIB.glb"))),
        Transform::IDENTITY
            .with_scale(Vec3::splat(1.0))
            .with_translation(Vec3::new(2.0, 0.5, 2.0)),
    ));

    if config.scene.spawn_outposts {
        commands.spawn((
            RigidBody::Static,
            WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("OUTPOST.glb"))),
            Transform::IDENTITY,
            ScanOutpost,
        ));
    }

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("TECH_CORE.glb"))),
        Transform::IDENTITY,
        TechCoreRoot,
        PreciousCollision(HashMap::from([(
            "GROUND".to_string(),
            (
                trimesh(),
                layer_env,
                Visibility::Visible,
                Some(RigidBody::Static),
            ),
        )])),
    ));

    if config.scene.spawn_power_runes {
        let mut power_rune_col = HashMap::from([(
            "BASE".to_string(),
            (
                trimesh(),
                layer_env,
                Visibility::Visible,
                Some(RigidBody::Static),
            ),
        )]);
        for i in 1..=2 {
            for j in 1..=5 {
                for k in ["ACTIVATED", "ACTIVE", "COMPLETED", "DISABLED"] {
                    power_rune_col.insert(
                        format!("FACE_{}_TARGET_{}_{}", i, j, k).to_string(),
                        (voxel(0.015), layer_env, Visibility::Visible, None),
                    );
                }
            }
        }
        commands.spawn((
            RigidBody::Static,
            CollisionMargin(0.001),
            Restitution::ZERO,
            WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("POWER.glb"))),
            Transform::IDENTITY,
            PowerRuneRoot,
            PreciousCollision(power_rune_col),
        ));
    }

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("vehicle.glb"))),
        // Keep the controlled vehicle clear of the outpost and dense ground
        // geometry rooted at the world origin. The blue target at (1, 0, 1)
        // remains roughly three metres in front of the initial camera.
        Transform::from_translation(CONTROLLED_VEHICLE_SPAWN),
        Infantry::new(Team::Red, INFANTRY_THREE_CONFIG),
        Controlled,
    ));

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("vehicle.glb"))),
        target_spawn_transform(PRIMARY_TARGET_SPAWN),
        Infantry::new(Team::Blue, INFANTRY_THREE_CONFIG),
        SlapperInfantry,
    ));

    commands.spawn((
        WorldAssetRoot(asset_server.load(GltfAssetLabel::Scene(0).from_asset("HERO.glb"))),
        target_spawn_transform(SECONDARY_TARGET_SPAWN),
        Infantry::new(Team::Blue, HERO_ROBOT_CONFIG),
        SlapperInfantry,
        ActiveSlapper,
    ));

    let mut main_camera = commands.spawn((
        Camera3d::default(),
        Camera {
            // When Talos/ROS2 capture is enabled, the actual on-screen preview is a UI blit of the
            // off-screen capture texture. Keep this camera inactive to avoid rendering twice.
            #[cfg(any(feature = "ros2", feature = "talos"))]
            is_active: false,
            #[cfg(not(any(feature = "ros2", feature = "talos")))]
            is_active: config.preview.enabled,
            // clear_color: ClearColorConfig::Custom(Color::BLACK),
            ..default()
        },
        Projection::Perspective(PerspectiveProjection {
            fov: config.camera.fov.to_radians(),
            near: 0.1,
            far: 500000000.0,
            ..default()
        }),
        Tonemapping::None,
        Msaa::Off,
        Transform::from_xyz(0.0, 10.0, 15.0).looking_at(Vec3::new(0.0, 0.0, 0.0), Vec3::Y),
        MainCamera {
            follow_offset: Vec3::from_array(config.camera.follow_offset),
        },
    ));
    if cfg!(target_os = "macos") && config.render.metalfx_temporal {
        main_camera.insert(MetalFxTemporalUpscaling {
            scale_factor: config.render.metalfx_scale,
            frame_generation: config.render.metalfx_frame_generation,
            reset: true,
        });
    } else if config.render.main_camera_fxaa {
        main_camera.insert(Fxaa::default());
    }
    if config.debug.egui {
        main_camera.insert(PrimaryEguiContext);
    }
    #[cfg(any(feature = "ros2", feature = "talos"))]
    main_camera.insert(crate::capture::CaptureSource);
}

fn target_spawn_transform(spawn: Vec3) -> Transform {
    Transform::from_translation(spawn).looking_at(CONTROLLED_VEHICLE_SPAWN, Vec3::Y)
}

pub fn setup_ground(
    events: On<WorldInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name>,
    ground: Query<Entity, With<ScanOutpost>>,
) {
    let root = events.entity;
    if !ground.contains(root) {
        return;
    }
    children.iter_descendants(root).for_each(|e| {
        let Ok(name) = name.get(e) else {
            return;
        };
        if name.as_str() == "OUTPOST_1" {
            commands.entity(e).insert(OutpostRoot::new(Team::Red));
        }
        if name.as_str() == "OUTPOST_2" {
            commands.entity(e).insert(OutpostRoot::new(Team::Blue));
        }
    })
}

pub fn setup_dart_launch(
    events: On<WorldInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name>,
    ground: Query<(), With<GroundRoot>>,
) {
    if ground.get(events.entity).is_err() {
        return;
    }

    for entity in children.iter_descendants(events.entity) {
        let Ok(name) = name.get(entity) else {
            continue;
        };
        if name.as_str() == "DART_LAUNCH_DIRECTION" {
            commands.entity(entity).insert(DartLaunch);
            return;
        }
    }

    warn!("GROUND.glb is missing DART_LAUNCH_DIRECTION");
}

pub fn setup_vehicle(
    events: On<WorldInstanceReady>,
    mut commands: Commands,
    query: HierarchyQuery,
    root_query: Query<(
        Entity,
        &Infantry,
        Option<&Controlled>,
        Option<&ActiveSlapper>,
    )>,
    _secondary_query: Query<&ChildOf, (Without<Infantry>, Without<WorldInstance>)>,
    _node_query: Query<(&Name, &ChildOf), (Without<Infantry>, Without<WorldInstance>)>,
    sim_config: Res<SimulationConfig>,
) {
    let root = events.entity;
    if root_query.get(root).is_err() {
        return;
    }
    let (root, infantry, is_local, is_active) = root_query.get(root).unwrap();
    let team = infantry.team;
    let config = infantry.config;
    let is_local = is_local.is_some();
    let is_active = is_active.is_some();
    if is_local {
        query.children.iter_descendants(root).for_each(|e| {
            commands.entity(e).insert(Controlled);
        });
    } else {
        query.children.iter_descendants(root).for_each(|e| {
            commands.entity(e).insert(SlapperInfantry);
            if is_active {
                commands.entity(e).insert(ActiveSlapper);
            }
        });
    }
    let vehicle_body_collision_layers = GameLayer::vehicle_body_collision_layers(is_local);
    let vehicle_armor_collision_layers = GameLayer::vehicle_armor_collision_layers(is_local);

    commands.entity(root).insert((
        RigidBody::Dynamic,
        VehicleDynamic::new(
            sim_config.vehicle.max_speed,
            sim_config.vehicle.linear_acceleration,
            sim_config.vehicle.acceleration_exponent,
        ),
        Collider::compound(vec![(
            Vec3::new(0.0, VEHICLE_COLLIDER_Y_OFFSET, 0.0),
            Quat::IDENTITY,
            Collider::cylinder(VEHICLE_COLLIDER_RADIUS, 0.231298),
        )]),
        CollisionMargin(0.005),
        vehicle_body_collision_layers,
        Mass(15.0),
        Restitution::new(0.01),
        AngularDamping(50.0),
    ));

    query.children.iter_descendants(root).for_each(|e| {
        commands.entity(e).insert(vehicle_armor_collision_layers);
    });

    let iter = query.of(root).any().exact("VEHICLE").flatten();
    let base = iter.clone().exact("BASE").one().unwrap();
    commands.entity(base).insert((
        InfantryChassis::default(),
        ScanArmor::new(team, config.armor),
    ));
    let gimbal = iter.exact("GIMBAL").one().unwrap();
    commands.entity(gimbal).insert(InfantryGimbal::default());
    if is_local {
        let q = query.of(gimbal).flatten();
        commands
            .entity(q.clone().exact("SHOT_DIRECTION").one().unwrap())
            .insert(InfantryLaunchOffset);
        commands
            .entity(q.exact("CAM_DIRECTION").one().unwrap())
            .insert(InfantryViewOffset);
    }
}

pub fn setup_collision(
    events: On<WorldInstanceReady>,
    mut commands: Commands,
    children: Query<&Children>,
    name: Query<&Name, With<Children>>,
    root_query: Query<(Entity, &PreciousCollision)>,
) {
    let Ok((_, PreciousCollision(map))) = root_query.get(events.entity) else {
        return;
    };
    for e in children.iter_descendants(events.entity) {
        let Ok(name) = name.get(e) else {
            continue;
        };
        if let Some((constructor, layer, visibility, rigid)) = map.get(&name.to_string()) {
            if let Some(rigid) = rigid {
                commands
                    .entity(e)
                    .insert((*rigid, constructor.clone(), *layer));
            } else {
                commands.entity(e).insert((constructor.clone(), *layer));
            }
            if visibility == &Visibility::Hidden {
                commands.entity(e).insert(*visibility);
            }
        }
    }
    commands.entity(events.entity).remove::<PreciousCollision>();
}

fn recovered_upright_rotation(rotation: Quat) -> Option<Quat> {
    let up = rotation * Vec3::Y;
    if up.dot(Vec3::Y) >= VEHICLE_RECOVERY_UP_DOT {
        return None;
    }

    let forward = rotation * Vec3::NEG_Z;
    let flat_forward = Vec2::new(forward.x, forward.z);
    let yaw = if flat_forward.length_squared() > 1e-6 {
        (-forward.x).atan2(-forward.z)
    } else {
        rotation.to_euler(EulerRot::YXZ).0
    };
    Some(Quat::from_rotation_y(yaw))
}

pub fn recover_tipped_vehicles(
    mut vehicles: Query<(&mut Transform, &mut AngularVelocity), With<Infantry>>,
) {
    for (mut transform, mut angular_velocity) in &mut vehicles {
        let Some(recovered_rotation) = recovered_upright_rotation(transform.rotation) else {
            continue;
        };
        transform.rotation = recovered_rotation;
        transform.translation.y += VEHICLE_RECOVERY_LIFT;
        angular_velocity.x = 0.0;
        angular_velocity.z = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        CONTROLLED_VEHICLE_SPAWN, PRIMARY_TARGET_SPAWN, VEHICLE_COLLIDER_RADIUS,
        VEHICLE_COLLIDER_Y_OFFSET, VEHICLE_SPAWN_HEIGHT, recovered_upright_rotation,
        target_spawn_transform,
    };
    use bevy::prelude::*;

    #[test]
    fn vehicle_spawn_height_starts_close_to_the_flat_ground() {
        let collider_bottom =
            VEHICLE_SPAWN_HEIGHT + VEHICLE_COLLIDER_Y_OFFSET - VEHICLE_COLLIDER_RADIUS;
        assert!(
            (0.10..=0.15).contains(&collider_bottom),
            "vehicle should start slightly above the ground before settling"
        );
    }

    #[test]
    fn controlled_vehicle_spawns_clear_of_the_outpost_origin() {
        assert!(
            CONTROLLED_VEHICLE_SPAWN.xz().length() >= 3.0,
            "controlled vehicle must not overlap origin-rooted world geometry"
        );
    }

    #[test]
    fn primary_target_starts_unobstructed_in_front_of_controlled_vehicle() {
        let offset = PRIMARY_TARGET_SPAWN - CONTROLLED_VEHICLE_SPAWN;
        assert!((0.5..=0.8).contains(&offset.x));
        assert!((-2.0..=-1.5).contains(&offset.z));
        assert!((1.7..=1.9).contains(&offset.length()));
    }

    #[test]
    fn primary_target_front_faces_controlled_vehicle() {
        let transform = target_spawn_transform(PRIMARY_TARGET_SPAWN);
        let forward = transform.rotation * Vec3::NEG_Z;
        let to_controlled = (CONTROLLED_VEHICLE_SPAWN - PRIMARY_TARGET_SPAWN).normalize();

        assert!(forward.dot(to_controlled) > 0.999);
    }

    #[test]
    fn ordinary_slope_does_not_trigger_recovery() {
        let rotation = Quat::from_rotation_x(30.0_f32.to_radians());

        assert!(recovered_upright_rotation(rotation).is_none());
    }

    #[test]
    fn severe_tilt_recovers_upright_and_preserves_heading() {
        let yaw = 35.0_f32.to_radians();
        let rotation = Quat::from_rotation_y(yaw) * Quat::from_rotation_x(75.0_f32.to_radians());

        let recovered = recovered_upright_rotation(rotation).unwrap();
        let recovered_up = recovered * Vec3::Y;
        let recovered_forward = recovered * Vec3::NEG_Z;
        let recovered_yaw = (-recovered_forward.x).atan2(-recovered_forward.z);

        assert!(recovered_up.abs_diff_eq(Vec3::Y, 1e-5));
        assert!((recovered_yaw - yaw).abs() < 1e-5);
    }
}
