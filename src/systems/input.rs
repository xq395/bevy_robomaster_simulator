use bevy::prelude::*;
use std::sync::atomic::Ordering;

use crate::components::{
    ActiveSlapper, Controlled, Infantry, InfantryChassis, InfantryGimbal, SlapperInfantry,
    SubscribeAutoAim,
};
use crate::config::SimulationConfig;
use crate::robomaster::vehicle::movement::VehicleDynamic;
use crate::systems::ControllerState;
use avian3d::prelude::*;

const CHASSIS_ROTATION_RESPONSE: f32 = 40.0;
const CHASSIS_ROTATION_STOP_EPSILON: f32 = 1e-3;
const CHASSIS_TILT_LIMIT: f32 = 20.0 * std::f32::consts::PI / 180.0;

fn update_chassis_rotation(
    chassis_transform: &mut Transform,
    chassis_data: &mut InfantryChassis,
    yaw_input: f32,
    roll_input: f32,
    pitch_input: f32,
    yaw_rotation_speed: f32,
    tilt_rotation_speed: f32,
    dt: f32,
) {
    let target_yaw_velocity = yaw_input * yaw_rotation_speed;
    let response = 1.0 - (-CHASSIS_ROTATION_RESPONSE * dt).exp();
    chassis_data.yaw_velocity += (target_yaw_velocity - chassis_data.yaw_velocity) * response;

    if chassis_data.yaw_velocity.abs() < CHASSIS_ROTATION_STOP_EPSILON
        && target_yaw_velocity.abs() < CHASSIS_ROTATION_STOP_EPSILON
    {
        chassis_data.yaw_velocity = 0.0;
    }

    chassis_data.yaw += chassis_data.yaw_velocity * dt;
    chassis_data.roll = (chassis_data.roll + roll_input * tilt_rotation_speed * dt)
        .clamp(-CHASSIS_TILT_LIMIT, CHASSIS_TILT_LIMIT);
    chassis_data.pitch = (chassis_data.pitch + pitch_input * tilt_rotation_speed * dt)
        .clamp(-CHASSIS_TILT_LIMIT, CHASSIS_TILT_LIMIT);
    chassis_transform.rotation = Quat::from_euler(
        EulerRot::YXZ,
        chassis_data.yaw,
        chassis_data.pitch,
        chassis_data.roll,
    );
}

pub fn vehicle_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    config: Res<SimulationConfig>,
    infantry: Single<(Forces, &Mass, &mut VehicleDynamic), (With<Infantry>, With<Controlled>)>,
    gimbal: Single<
        (&GlobalTransform, &InfantryGimbal),
        (With<Controlled>, Without<InfantryChassis>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (
            With<Controlled>,
            Without<InfantryGimbal>,
            With<InfantryChassis>,
            Without<Infantry>,
        ),
    >,
) {
    let controller = controller.controlled;
    let input = controller.movement;
    let boost = controller.boost_multiplier();

    let (mut forces, &Mass(mass), mut dynamic) = infantry.into_inner();

    let dt = time.delta_secs();
    dynamic.linear(
        &mut forces,
        mass,
        gimbal.into_inner().0,
        input,
        time.delta_secs(),
        boost,
    );

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    update_chassis_rotation(
        &mut chassis_transform,
        &mut chassis_data,
        controller.chassis_yaw,
        controller.chassis_roll,
        controller.chassis_pitch,
        config.vehicle.rotation_speed,
        config.vehicle.tilt_rotation_speed,
        dt,
    );
}

pub fn remote_vehicle_controls(
    time: Res<Time>,
    controller: Res<ControllerState>,
    config: Res<SimulationConfig>,
    infantry: Single<
        (&GlobalTransform, Forces, &Mass, &mut VehicleDynamic),
        (With<ActiveSlapper>, With<Infantry>, Without<Controlled>),
    >,
    chassis: Single<
        (&mut Transform, &mut InfantryChassis),
        (
            With<ActiveSlapper>,
            With<InfantryChassis>,
            Without<InfantryGimbal>,
            Without<Infantry>,
        ),
    >,
) {
    let controller = controller.remote;
    let input = controller.movement;
    let boost = controller.boost_multiplier();

    let (infantry_global_transform, mut forces, &Mass(mass), mut dynamic) = infantry.into_inner();

    let dt = time.delta_secs();
    dynamic.linear(
        &mut forces,
        mass,
        infantry_global_transform,
        input,
        time.delta_secs(),
        boost,
    );

    let (mut chassis_transform, mut chassis_data) = chassis.into_inner();
    update_chassis_rotation(
        &mut chassis_transform,
        &mut chassis_data,