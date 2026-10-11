use avian3d::prelude::{CollisionEnd, CollisionEventsEnabled};
use bevy::prelude::{ChildOf, Commands, Entity, On, Plugin, Query, ResMut, Resource, Update, With};
use std::collections::HashSet;

use super::construct::Armor;
use crate::robomaster::power_rune::prelude::Projectile;
use crate::statistic::ProjectileStatistics;

#[derive(Resource, Default)]
struct ConsumedArmorProjectiles(HashSet<Entity>);

fn mark_projectile_consumed(consumed: &mut HashSet<Entity>, projectile: Entity) -> bool {
    consumed.insert(projectile)
}

fn handle_armor_collision(
    event: On<CollisionEnd>,
    mut commands: Commands,
    mut consumed_projectiles: ResMut<ConsumedArmorProjectiles>,
    mut stats: ResMut<ProjectileStatistics>,
    projectiles: Query<Entity, With<Projectile>>,
    armors: Query<(), With<Armor>>,
    child_of: Query<&ChildOf>,
) {
    let projectile_body1 = event.body1.and_then(|body| projectiles.get(body).ok());
    let projectile_body2 = event.body2.and_then(|body| projectiles.get(body).ok());

    let projectile_entity = match (projectile_body1, projectile_body2) {
        (Some(e), _) => e,
        (_, Some(e)) => e,
        _ => return,
    };

    let other_collider = if projectile_body1.is_some() {
        event.collider2
    } else {
        event.collider1
    };

    if armors.contains(other_collider)
        || child_of
            .iter_ancestors(other_collider)
            .any(|ancestor| armors.contains(ancestor))
    {
        // Commands are deferred, so the marker removal below cannot prevent another
        // CollisionEnd observer from seeing the same projectile in this frame.
        if !mark_projectile_consumed(&mut consumed_projectiles.0, projectile_entity) {
            return;
        }

        commands
            .entity(projectile_entity)
            .remove::<CollisionEventsEnabled>();
        stats.increase_armor_hit();
    }
}

fn cleanup_consumed_armor_projectiles(
    mut consumed_projectiles: ResMut<ConsumedArmorProjectiles>,
    projectiles: Query<(), With<Projectile>>,
) {
    consumed_projectiles
        .0
        .retain(|entity| projectiles.contains(*entity));
}

#[derive(Default)]
pub(super) struct ArmorCollisionPlugin;

impl Plugin for ArmorCollisionPlugin {
    fn build(&self, app: &mut bevy::app::App) {
        app.init_resource::<ConsumedArmorProjectiles>()
            .add_systems(Update, cleanup_consumed_armor_projectiles)
            .add_observer(handle_armor_collision);
    }
}

#[cfg(test)]
mod tests {
    use super::mark_projectile_consumed;
    use bevy::prelude::World;
    use std::collections::HashSet;

    #[test]
    fn a_projectile_can_only_be_consumed_once() {
        let mut world = World::new();
        let projectile = world.spawn_empty().id();
        let mut consumed = HashSet::new();

        assert!(mark_projectile_consumed(&mut consumed, projectile));
        assert!(!mark_projectile_consumed(&mut consumed, projectile));
    }
}
