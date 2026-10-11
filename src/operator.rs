//! Simulator-side operator authority. UDP is a local development adapter, not RM wire protocol.
use bevy::prelude::*;
use serde::{Deserialize, Serialize};
use std::net::UdpSocket;
use std::time::{SystemTime, UNIX_EPOCH};

pub fn now_ms() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_millis() as u64
}

#[derive(SystemSet, Debug, Clone, PartialEq, Eq, Hash)]
pub enum Execution { Receive, Fire }

#[derive(Default, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ControlMode { #[default] Command, Manual }

#[derive(Default, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ManualInput {
    pub movement: [f32; 2],
    pub gimbal: [f32; 2],
    pub shoot: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControlCommand {
    session: u64,
    seq: u64,
    sent_ms: u64,
    ttl_ms: u64,
    aim_enabled: bool,
    fire_authorized: bool,
    spin_enabled: bool,
    #[serde(default)]
    mode: ControlMode,
    #[serde(default)]
    manual: ManualInput,
}

#[derive(Resource, Serialize)]
pub struct OperatorState {
    pub session: u64,
    pub applied_seq: u64,
    pub remote: bool,
    pub mode: ControlMode,
    pub manual: ManualInput,
    pub aim_enabled: bool,
    pub fire_authorized: bool,
    pub spin_enabled: bool,
    pub reason: String,
    pub shot_count: u64,
    pub position: [f32; 2],
    pub chassis_heading: f32,
    pub gimbal_heading: f32,
    #[serde(skip)]
    expires_ms: u64,
    #[serde(skip)]
    pub automatic_request: bool,
    #[serde(skip)]
    pub accept_after_ns: u64,
    #[serde(skip)]
    pub last_command_ns: u64,
}

impl Default for OperatorState {
    fn default() -> Self {
        Self { session: now_ms(), applied_seq: 0, remote: false, mode: ControlMode::Command,
            manual: ManualInput::default(), aim_enabled: false,
            fire_authorized: false, spin_enabled: false, reason: "idle".into(),
            shot_count: 0, position: [0.0; 2], chassis_heading: 0.0, gimbal_heading: 0.0,
            expires_ms: 0, automatic_request: false,
            accept_after_ns: 0, last_command_ns: 0 }
    }
}

impl OperatorState {
    pub fn fire_requested(&self, manual_pressed: bool) -> bool {
        if self.remote && self.mode == ControlMode::Command {
            return self.aim_enabled && self.fire_authorized && self.automatic_request;
        }
        if self.aim_enabled {
            self.fire_authorized && self.automatic_request
        } else {
            manual_pressed
        }
    }
    pub fn revoke(&mut self) {
        self.session = self.session.saturating_add(1);
        self.applied_seq = 0;
        self.remote = false;
        self.mode = ControlMode::Command;
        self.manual = ManualInput::default();
        self.aim_enabled = false;
        self.fire_authorized = false;
        self.spin_enabled = false;
        self.automatic_request = false;
        self.accept_after_ns = now_ms() * 1_000_000;
    }

    fn apply(&mut self, cmd: ControlCommand, now: u64) -> Result<(), &'static str> {
        if cmd.session != self.session { return Err("session_mismatch"); }
        if cmd.seq <= self.applied_seq { return Err("duplicate_or_reordered"); }
        if !(1..=500).contains(&cmd.ttl_ms) || cmd.sent_ms > now ||
            now.saturating_sub(cmd.sent_ms) >= cmd.ttl_ms { return Err("expired_command"); }
        if cmd.fire_authorized && !cmd.aim_enabled { return Err("authorization_requires_aim"); }
        if cmd.manual.movement.iter().chain(cmd.manual.gimbal.iter()).any(|v| !v.is_finite() || v.abs() > 1.0) {
            return Err("invalid_manual_input");
        }
        if cmd.mode == ControlMode::Manual && (cmd.aim_enabled || cmd.fire_authorized || cmd.spin_enabled) {
            return Err("manual_requires_automation_off");
        }
        if cmd.mode == ControlMode::Command && (cmd.manual.shoot || cmd.manual.movement != [0.0; 2] || cmd.manual.gimbal != [0.0; 2]) {
            return Err("manual_input_in_command_mode");
        }
        if self.aim_enabled != cmd.aim_enabled || self.fire_authorized != cmd.fire_authorized {
            self.accept_after_ns = now * 1_000_000;
        }
        self.applied_seq = cmd.seq;
        self.remote = true;
        self.mode = cmd.mode;
        self.manual = cmd.manual;
        self.aim_enabled = cmd.aim_enabled;
        self.fire_authorized = cmd.fire_authorized;
        self.spin_enabled = cmd.spin_enabled;
        self.expires_ms = cmd.sent_ms + cmd.ttl_ms;
        Ok(())
    }

    pub fn accept_command(&mut self, stamp: u64, now: u64) -> bool {
        if stamp <= self.last_command_ns || stamp <= self.accept_after_ns || stamp > now ||
            now.saturating_sub(stamp) > 200_000_000 {
            self.reason = "stale_or_repeated_aim_command".into();
            return false;
        }
        self.last_command_ns = stamp;
        true
    }
}

#[derive(Resource, Default)]
pub struct OperatorSocket(pub Option<UdpSocket>);

pub fn setup(mut commands: Commands) {
    let socket = UdpSocket::bind("127.0.0.1:5001").and_then(|s| { s.set_nonblocking(true)?; Ok(s) });
    match socket {
        Ok(s) => { commands.insert_resource(OperatorSocket(Some(s))); }
        Err(e) => { warn!("Operator UDP disabled: {e}"); commands.insert_resource(OperatorSocket(None)); }
    }
}

pub fn update(
    mut state: ResMut<OperatorState>, socket: Res<OperatorSocket>,
    mut controller: ResMut<crate::systems::ControllerState>,
    keys: Res<ButtonInput<KeyCode>>, windows: Query<&Window>,
) {
    let now = now_ms();
    state.automatic_request = false;
    let takeover = keys.just_pressed(KeyCode::Escape);
    let local_unfocused = !state.remote && windows.iter().any(|w| !w.focused);
    let expired = state.remote && now >= state.expires_ms;
    if takeover || expired || (local_unfocused && (state.aim_enabled || state.fire_authorized || state.spin_enabled)) {
        state.revoke();
        controller.set_operator_modes(false, false);
        controller.controlled.shoot = false;
        state.reason = if takeover { "manual_takeover" } else { "operator_timeout" }.into();
    }
    if local_unfocused {
        controller.set_operator_modes(false, false);
        controller.controlled.shoot = false;
    }
    if let Some(socket) = &socket.0 {
        let mut buf = [0u8; 2048];
        // Bound work per frame; old packets retain their sender-side expiry.
        for _ in 0..32 {
            let Ok((len, peer)) = socket.recv_from(&mut buf) else { break; };
            if &buf[..len] != b"status" {
                let result = if takeover { Err("manual_takeover") } else {
                    serde_json::from_slice::<ControlCommand>(&buf[..len])
                        .map_err(|_| "invalid_command").and_then(|c| state.apply(c, now))
                };
                state.reason = result.err().unwrap_or("applied").into();
            }
            if let Ok(bytes) = serde_json::to_vec(&*state) { let _ = socket.send_to(&bytes, peer); }
        }
    }
    if state.remote {
        controller.set_operator_modes(state.aim_enabled, state.spin_enabled);
        controller.controlled.shoot = false;
        if state.mode == ControlMode::Manual {
            let switch_camera = controller.controlled.switch_camera_just_pressed;
            controller.controlled = crate::systems::ControllerInput::default();
            controller.controlled.switch_camera_just_pressed = switch_camera;
            controller.controlled.movement = Vec2::from_array(state.manual.movement);
            controller.controlled.gimbal = Vec2::from_array(state.manual.gimbal);
            controller.controlled.shoot = state.manual.shoot;
        }
    } else {
        let aim = controller.auto_aim_active();
        let fire = aim && controller.controlled.shoot;
        if aim != state.aim_enabled || fire != state.fire_authorized {
            state.accept_after_ns = now * 1_000_000;
        }
        state.aim_enabled = aim;
        state.fire_authorized = fire;
        state.spin_enabled = controller.controlled_chassis_spin();
    }
}

pub fn update_pose(
    mut state: ResMut<OperatorState>,
    chassis: Query<&GlobalTransform, (With<crate::components::Controlled>, With<crate::components::InfantryChassis>)>,
    gimbal: Query<&GlobalTransform, (With<crate::components::Controlled>, With<crate::components::InfantryGimbal>)>,
) {
    if let Ok(tf) = chassis.single() {
        state.position = [tf.translation().x, tf.translation().z];
        let forward = tf.rotation() * Vec3::NEG_Z;
        state.chassis_heading = forward.x.atan2(-forward.z);
    }
    if let Ok(tf) = gimbal.single() {
        let forward = tf.rotation() * Vec3::NEG_Z;
        state.gimbal_heading = forward.x.atan2(-forward.z);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn cmd(s: &OperatorState, seq: u64) -> ControlCommand {
        ControlCommand { session: s.session, seq, sent_ms: 1000, ttl_ms: 200,
            aim_enabled: true, fire_authorized: true, spin_enabled: false,
            mode: ControlMode::Command, manual: ManualInput::default() }
    }
    #[test]
    fn replay_expiry_and_session_revocation() {
        let mut s = OperatorState::default();
        assert!(s.apply(cmd(&s, 1), 1050).is_ok());
        assert_eq!(s.apply(cmd(&s, 1), 1051), Err("duplicate_or_reordered"));
        assert_eq!(s.apply(cmd(&s, 2), 1200), Err("expired_command"));
        let old = cmd(&s, 3);
        s.revoke();
        assert_eq!(s.apply(old, 1100), Err("session_mismatch"));
        assert!(!s.fire_authorized);
    }

    #[test]
    fn manual_takeover_excludes_automation_and_validates_axes() {
        let mut s = OperatorState::default();
        let mut c = cmd(&s, 1);
        c.mode = ControlMode::Manual;
        assert_eq!(s.apply(c, 1050), Err("manual_requires_automation_off"));
        let mut c = cmd(&s, 1);
        c.mode = ControlMode::Manual;
        c.aim_enabled = false;
        c.fire_authorized = false;
        c.manual.movement = [0.0, 1.0];
        c.manual.shoot = true;
        assert!(s.apply(c, 1050).is_ok());
        assert!(!s.aim_enabled && s.manual.shoot);
        s.revoke();
        assert!(!s.manual.shoot && s.manual.movement == [0.0; 2]);
        let mut c = cmd(&s, 1);
        c.manual.gimbal = [f32::NAN, 0.0];
        assert_eq!(s.apply(c, 1050), Err("invalid_manual_input"));
    }
    #[test]
    fn old_aim_cannot_cross_authorization_boundary() {
        let mut s = OperatorState::default();
        s.accept_after_ns = 100;
        assert!(!s.accept_command(99, 200));
        assert!(s.accept_command(150, 200));
        assert!(!s.accept_command(150, 201));
        assert!(!s.accept_command(300, 200));
        assert!(!s.accept_command(200, 300_000_000));
    }

    #[test]
    fn automatic_fire_requires_both_authorization_and_advice() {
        let mut s = OperatorState::default();
        assert!(s.fire_requested(true));
        s.aim_enabled = true;
        s.automatic_request = true;
        assert!(!s.fire_requested(true)); // manual button cannot bypass FireGate
        s.fire_authorized = true;
        assert!(s.fire_requested(false));
        s.automatic_request = false;
        assert!(!s.fire_requested(true));
        s.revoke();
        assert!(!s.fire_requested(false));
    }

    #[test]
    fn expired_remote_lease_clears_controller_and_rejects_old_session() {
        let mut app = App::new();
        let mut s = OperatorState::default();
        let old_session = s.session;
        s.remote = true;
        s.aim_enabled = true;
        s.fire_authorized = true;
        s.spin_enabled = true;
        s.expires_ms = 1;
        app.insert_resource(s)
            .init_resource::<OperatorSocket>()
            .init_resource::<crate::systems::ControllerState>()
            .init_resource::<ButtonInput<KeyCode>>()
            .add_systems(Update, update);
        app.update();
        let s = app.world().resource::<OperatorState>();
        assert!(!s.remote && !s.fire_authorized && !s.aim_enabled && !s.spin_enabled);
        assert_ne!(s.session, old_session);
        let controller = app.world().resource::<crate::systems::ControllerState>();
        assert!(!controller.auto_aim_active());
        assert!(!controller.controlled_chassis_spin());
    }

    #[test]
    fn local_authorization_is_hold_to_run_and_escape_takes_over() {
        let mut app = App::new();
        app.init_resource::<OperatorState>()
            .init_resource::<OperatorSocket>()
            .init_resource::<crate::systems::ControllerState>()
            .init_resource::<ButtonInput<KeyCode>>()
            .add_systems(Update, update);
        {
            let mut c = app.world_mut().resource_mut::<crate::systems::ControllerState>();
            c.set_operator_modes(true, true);
            c.controlled.shoot = true;
        }
        app.update();
        assert!(app.world().resource::<OperatorState>().fire_authorized);
        app.world_mut().resource_mut::<crate::systems::ControllerState>().controlled.shoot = false;
        app.update();
        assert!(!app.world().resource::<OperatorState>().fire_authorized);
        assert!(app.world().resource::<OperatorState>().spin_enabled);
        app.world_mut().resource_mut::<ButtonInput<KeyCode>>().press(KeyCode::Escape);
        app.update();
        assert!(!app.world().resource::<OperatorState>().aim_enabled);
    }
}