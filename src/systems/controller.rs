use bevy::input::gamepad::{GamepadRumbleIntensity, GamepadRumbleRequest};
use bevy::prelude::*;
use core::time::Duration;
use std::sync::atomic::Ordering;

use crate::components::SubscribeAutoAim;

const GAMEPAD_STICK_DEADZONE: f32 = 0.12;
const GAMEPAD_TRIGGER_THRESHOLD: f32 = 0.35;
const PRECISE_GIMBAL_SCALE: f32 = 0.35;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControllerHelp {
    source: &'static str,
    manual: &'static str,
    auto_aim: &'static str,
}

impl ControllerHelp {
    const fn keyboard() -> Self {
        Self {
            source: "keyboard",
            manual: "F3 Camera | WASD Move | Arrows Aim | Space Shoot | G Dart | Q Gyro | U Remote Gyro | F5 AutoAim | Tab Slapper",
            auto_aim: "F5 AutoAim Off | WASD Move | Q Gyro | U Remote Gyro | hold Space: authorize automatic fire | Esc: take over | Tab Slapper",
        }
    }

    const fn xbox() -> Self {
        Self {
            source: "xbox",
            manual: "View Camera | LS Move | L3 Boost | DPad Slapper Move | RS Aim | R3+RS Slapper Roll/Pitch | LB Gyro | Y Slapper Gyro | RB Shoot | X Dart | hold RT AutoAim",
            auto_aim: "release RT AutoAim Off | LS Move | L3 Boost | DPad Slapper Move | R3+RS Slapper Roll/Pitch | LB Gyro | Y Slapper Gyro | hold RB: authorize automatic fire",
        }
    }
}

impl Default for ControllerHelp {
    fn default() -> Self {
        Self::keyboard()
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum ChassisSpinMode {
    #[default]
    Off,
    On,
}

impl ChassisSpinMode {
    fn toggle(&mut self) {
        *self = match self {
            Self::Off => Self::On,
            Self::On => Self::Off,
        };
    }

    fn yaw_input(self) -> f32 {
        match self {
            Self::Off => 0.0,
            Self::On => 1.0,
        }
    }

    fn is_on(self) -> bool {
        self == Self::On
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ControllerInput {
    pub movement: Vec2,
    pub gimbal: Vec2,
    pub chassis_yaw: f32,
    pub chassis_roll: f32,
    pub chassis_pitch: f32,
    pub boost: bool,
    pub precise_gimbal: bool,
    pub shoot: bool,
    pub dart_just_pressed: bool,
    pub switch_slapper_just_pressed: bool,
    pub switch_camera_just_pressed: bool,
    pub auto_aim: bool,
}

impl Default for ControllerInput {
    fn default() -> Self {
        Self {
            movement: Vec2::ZERO,
            gimbal: Vec2::ZERO,
            chassis_yaw: 0.0,
            chassis_roll: 0.0,
            chassis_pitch: 0.0,
            boost: false,
            precise_gimbal: false,
            shoot: false,
            dart_just_pressed: false,
            switch_slapper_just_pressed: false,
            switch_camera_just_pressed: false,
            auto_aim: false,
        }
    }
}

impl ControllerInput {
    pub fn boost_multiplier(self) -> f32 {
        if self.boost { 2.0 } else { 1.0 }
    }

    pub fn gimbal_scale(self) -> f32 {
        if self.precise_gimbal {
            PRECISE_GIMBAL_SCALE
        } else {
            1.0
        }
    }

    fn add_movement(&mut self, movement: Vec2) {
        self.movement = clamp_axes_vec2(self.movement + movement);
    }

    fn add_gimbal(&mut self, gimbal: Vec2) {
        self.gimbal = clamp_axes_vec2(self.gimbal + gimbal);
    }

    fn add_chassis(&mut self, yaw: f32, roll: f32, pitch: f32) {
        self.chassis_yaw = (self.chassis_yaw + yaw).clamp(-1.0, 1.0);
        self.chassis_roll = (self.chassis_roll + roll).clamp(-1.0, 1.0);
        self.chassis_pitch = (self.chassis_pitch + pitch).clamp(-1.0, 1.0);
    }
}

#[derive(Resource, Debug, Default)]
pub struct ControllerState {
    pub controlled: ControllerInput,
    pub remote: ControllerInput,
    keyboard_auto_aim: bool,
    controlled_chassis_spin: ChassisSpinMode,
    remote_chassis_spin: ChassisSpinMode,
    active_gamepad: Option<Entity>,
    help: ControllerHelp,
}

impl ControllerState {
    pub fn set_operator_modes(&mut self, aim: bool, spin: bool) {
        self.keyboard_auto_aim = aim;
        self.controlled.auto_aim = aim;
        self.controlled_chassis_spin = if spin { ChassisSpinMode::On } else { ChassisSpinMode::Off };
        self.controlled.chassis_yaw = self.controlled_chassis_spin.yaw_input();
    }
    pub fn reset_frame(&mut self) {
        self.controlled = ControllerInput::default();
        self.remote = ControllerInput::default();
    }

    pub fn auto_aim_active(&self) -> bool {
        if self.active_gamepad.is_some() { self.controlled.auto_aim } else { self.keyboard_auto_aim }
    }

    pub fn help_source(&self) -> &'static str {
        self.help.source
    }

    pub fn help_mode(&self) -> &'static str {
        if self.auto_aim_active() {
            "auto-aim"
        } else {
            "manual"
        }
    }

    pub fn help_controls(&self) -> &'static str {
        if self.auto_aim_active() {
            self.help.auto_aim
        } else {
            self.help.manual
        }
    }

    pub fn controlled_chassis_spin(&self) -> bool {
        self.controlled_chassis_spin.is_on()
    }

    pub fn remote_chassis_spin(&self) -> bool {
        self.remote_chassis_spin.is_on()
    }

    pub fn active_gamepad(&self) -> Option<Entity> {
        self.active_gamepad
    }

    fn toggle_keyboard_auto_aim(&mut self) {
        self.keyboard_auto_aim = !self.keyboard_auto_aim;
    }

    fn toggle_controlled_chassis_spin(&mut self) {
        self.controlled_chassis_spin.toggle();
    }

    fn toggle_remote_chassis_spin(&mut self) {
        self.remote_chassis_spin.toggle();
    }

    fn use_help(&mut self, help: ControllerHelp) {
        self.help = help;
    }

    fn use_gamepad(&mut self, gamepad: Entity) {
        self.active_gamepad = Some(gamepad);
    }

    fn clear_gamepad(&mut self) {
        self.active_gamepad = None;
    }
}

pub fn clear_controller_input(mut controller: ResMut<ControllerState>) {
    controller.reset_frame();
}

pub fn sample_keyboard_controller(
    keyboard: Res<ButtonInput<KeyCode>>,
    mode: Res<crate::components::CameraMode>,
    mut controller: ResMut<ControllerState>,
) {
    if mode.0 == crate::components::FollowingType::Free {
        controller.controlled.switch_camera_just_pressed = keyboard.just_pressed(KeyCode::F3);
        controller.set_operator_modes(false, false);
        return;
    }
    let keyboard_used = keyboard_controller_active(&keyboard);
    if keyboard_used {
        controller.use_help(ControllerHelp::keyboard());
        controller.clear_gamepad();
    }
    if keyboard.just_pressed(KeyCode::KeyQ) {
        controller.toggle_controlled_chassis_spin();
    }
    if keyboard.just_pressed(KeyCode::KeyU) {
        controller.toggle_remote_chassis_spin();
    }

    let controlled_chassis_yaw = controller.controlled_chassis_spin.yaw_input();
    let controlled = &mut controller.controlled;
    controlled.add_movement(keyboard_vec2(
        &keyboard,
        KeyCode::KeyW,
        KeyCode::KeyA,
        KeyCode::KeyS,
        KeyCode::KeyD,
    ));
    controlled.add_chassis(controlled_chassis_yaw, 0.0, 0.0);
    controlled.add_gimbal(Vec2::new(
        keyboard_axis(&keyboard, KeyCode::ArrowLeft, KeyCode::ArrowRight),
        keyboard_axis(&keyboard, KeyCode::ArrowUp, KeyCode::ArrowDown),
    ));
    controlled.boost |= keyboard.pressed(KeyCode::ShiftLeft);
    controlled.shoot |= keyboard.pressed(KeyCode::Space);
    controlled.dart_just_pressed |= keyboard.just_pressed(KeyCode::KeyG);
    controlled.switch_slapper_just_pressed |= keyboard.just_pressed(KeyCode::Tab);
    controlled.switch_camera_just_pressed |= keyboard.just_pressed(KeyCode::F3);

    let remote_chassis_yaw = controller.remote_chassis_spin.yaw_input();
    let remote = &mut controller.remote;
    remote.add_movement(keyboard_vec2(
        &keyboard,
        KeyCode::KeyI,
        KeyCode::KeyJ,
        KeyCode::KeyK,
        KeyCode::KeyL,
    ));
    remote.add_chassis(
        remote_chassis_yaw,
        keyboard_axis(&keyboard, KeyCode::BracketLeft, KeyCode::BracketRight),
        keyboard_axis(&keyboard, KeyCode::Semicolon, KeyCode::Quote),
    );
    if !keyboard.pressed(KeyCode::ShiftLeft) {
        remote.add_gimbal(Vec2::new(
            keyboard_axis(&keyboard, KeyCode::KeyC, KeyCode::KeyB),
            0.0,
        ));
    }
    remote.add_gimbal(Vec2::new(
        0.0,
        keyboard_axis(&keyboard, KeyCode::KeyF, KeyCode::KeyV),
    ));
    remote.boost |= keyboard.pressed(KeyCode::ShiftRight);

    if keyboard.just_pressed(KeyCode::F5) {
        controller.toggle_keyboard_auto_aim();