use bevy::prelude::*;
use serde_json::json;
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

const PROJECTILE_STATISTICS_FILE: &str = "projectile_statistics.json";
const PROJECTILE_STATISTICS_RESET_REQUEST_FILE: &str = "projectile_statistics_reset.request";
const LEGACY_ARMOR_HIT_RESET_REQUEST_FILE: &str = "armor_hit_reset.request";

#[derive(Resource, Default, Reflect)]
#[reflect(Resource)]
pub struct ProjectileStatistics {
    pub launch_count: u32,
    pub accurate_count: u32,
    pub armor_hit_count: u32,
}

impl ProjectileStatistics {
    pub fn increase_launch(&mut self) {
        self.launch_count += 1;
    }

    pub fn increase_accurate(&mut self) {
        self.accurate_count += 1;
    }

    pub fn increase_armor_hit(&mut self) {
        self.armor_hit_count += 1;
        self.increase_accurate();
    }

    pub fn clear(&mut self) {
        self.launch_count = 0;
        self.accurate_count = 0;
        self.armor_hit_count = 0;
    }

    pub fn accurate_pct(&self) -> f32 {
        if self.launch_count == 0 {
            return 0.0;
        }
        (self.accurate_count as f32) / (self.launch_count as f32)
    }

    pub fn armor_hit_pct(&self) -> f32 {
        if self.launch_count == 0 {
            return 0.0;
        }
        (self.armor_hit_count as f32) / (self.launch_count as f32)
    }
}

#[derive(Default)]
pub(crate) struct ProjectileStatisticsWebState {
    directory: Option<PathBuf>,
    last_snapshot: Option<(u32, u32, u32)>,
}

pub fn sync_projectile_statistics_web(
    mut stats: ResMut<ProjectileStatistics>,
    mut state: Local<ProjectileStatisticsWebState>,
) {
    if state.directory.is_none() {
        state.directory = std::env::var_os("TALOS_SHM_DIR").map(PathBuf::from);
    }
    let Some(directory) = state.directory.as_ref() else {
        return;
    };

    for reset_request in [
        directory.join(PROJECTILE_STATISTICS_RESET_REQUEST_FILE),
        directory.join(LEGACY_ARMOR_HIT_RESET_REQUEST_FILE),
    ] {
        if reset_request.is_file() {
            match std::fs::remove_file(&reset_request) {
                Ok(()) => stats.clear(),
                Err(error) => warn!(
                    "cannot consume projectile statistics reset request {}: {}",
                    reset_request.display(),
                    error
                ),
            }
        }
    }

    let snapshot = (
        stats.launch_count,
        stats.accurate_count,
        stats.armor_hit_count,
    );
    if state.last_snapshot == Some(snapshot) {
        return;
    }

    let updated_unix_ms = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis())
        .unwrap_or_default();
    let payload = json!({
        "launch_count": stats.launch_count,
        "accurate_count": stats.accurate_count,
        "armor_hit_count": stats.armor_hit_count,
        "armor_hit_rate": stats.armor_hit_pct(),
        "updated_unix_ms": updated_unix_ms,
    });
    let output = directory.join(PROJECTILE_STATISTICS_FILE);
    match serde_json::to_vec(&payload)
        .map_err(std::io::Error::other)
        .and_then(|bytes| std::fs::write(&output, bytes))
    {
        Ok(()) => state.last_snapshot = Some(snapshot),
        Err(error) => warn!(
            "cannot publish projectile statistics {}: {}",
            output.display(),
            error
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::ProjectileStatistics;

    #[test]
    fn armor_hits_are_counted_separately_and_as_accurate_hits() {
        let mut stats = ProjectileStatistics::default();

        stats.increase_armor_hit();

        assert_eq!(stats.armor_hit_count, 1);
        assert_eq!(stats.accurate_count, 1);
    }

    #[test]
    fn non_armor_accurate_hits_do_not_change_the_armor_hit_count() {
        let mut stats = ProjectileStatistics::default();

        stats.increase_accurate();

        assert_eq!(stats.armor_hit_count, 0);
        assert_eq!(stats.accurate_count, 1);
    }

    #[test]
    fn clearing_statistics_resets_launches_and_hits_together() {
        let mut stats = ProjectileStatistics {
            launch_count: 8,
            accurate_count: 3,
            armor_hit_count: 2,
        };

        stats.clear();

        assert_eq!(stats.armor_hit_count, 0);
        assert_eq!(stats.launch_count, 0);
        assert_eq!(stats.accurate_count, 0);
    }

    #[test]
    fn armor_hit_percentage_uses_launches_as_denominator() {
        let stats = ProjectileStatistics {
            launch_count: 8,
            accurate_count: 5,
            armor_hit_count: 2,
        };

        assert_eq!(stats.armor_hit_pct(), 0.25);
    }
}
