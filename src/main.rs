use macroquad::{
    color::{GRAY, WHITE},
    input::{is_mouse_button_down, mouse_position, MouseButton},
    shapes::draw_circle,
    window::{clear_background, next_frame, screen_height, screen_width},
};
use rapier2d::{
    dynamics::{
        CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
        RigidBodyBuilder, RigidBodySet,
    },
    geometry::{BroadPhaseMultiSap, ColliderBuilder, ColliderSet, NarrowPhase},
    na::{vector, Vector2},
    pipeline::{PhysicsPipeline, QueryPipeline},
    prelude::{nalgebra, RigidBodyHandle},
};

const PHYSICS_SCALE: f32 = 50.0;

struct Ball {
    radius: f32,
    position: Vector2<f32>,
    handle: RigidBodyHandle,
}

#[macroquad::main("hello-macroquad-rapier")]
async fn main() {
    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();
    let mut balls: Vec<Ball> = vec![];

    add_wall(&mut collider_set);

    let gravity = vector![0.0, -9.81];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut island_manager = IslandManager::new();
    let mut broad_phase = BroadPhaseMultiSap::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut impulse_joint_set = ImpulseJointSet::new();
    let mut multibody_joint_set = MultibodyJointSet::new();
    let mut ccd_solver = CCDSolver::new();
    let mut query_pipeline = QueryPipeline::new();
    let physics_hooks = ();
    let event_handler = ();

    let mut clicked = false;

    loop {
        clear_background(GRAY);

        if is_mouse_button_down(MouseButton::Left) {
            if !clicked {
                clicked = true;
                let (mouse_x, mouse_y) = mouse_position();

                let ball = add_ball(mouse_x, mouse_y, &mut rigid_body_set, &mut collider_set);
                balls.push(ball);
            }
        } else {
            clicked = false;
        }

        for ball in balls.iter() {
            draw_circle(
                PHYSICS_SCALE * ball.position.x,
                -PHYSICS_SCALE * ball.position.y,
                PHYSICS_SCALE * ball.radius,
                WHITE,
            );
        }

        physics_pipeline.step(
            &gravity,
            &integration_parameters,
            &mut island_manager,
            &mut broad_phase,
            &mut narrow_phase,
            &mut rigid_body_set,
            &mut collider_set,
            &mut impulse_joint_set,
            &mut multibody_joint_set,
            &mut ccd_solver,
            Some(&mut query_pipeline),
            &physics_hooks,
            &event_handler,
        );

        for ball in balls.iter_mut() {
            let ball_body = &rigid_body_set[ball.handle];

            ball.position = *ball_body.translation();
        }

        next_frame().await
    }
}

fn add_wall(collider_set: &mut ColliderSet) {
    let wall_thickness = 0.05;

    let collider_bottom = ColliderBuilder::cuboid(screen_width() / PHYSICS_SCALE, wall_thickness)
        .translation(vector![
            0.0,
            (screen_height() / -PHYSICS_SCALE) - wall_thickness
        ])
        .build();
    collider_set.insert(collider_bottom);

    let collider_left = ColliderBuilder::cuboid(wall_thickness, screen_height() / PHYSICS_SCALE)
        .translation(vector![0.0, 0.0])
        .build();
    collider_set.insert(collider_left);

    let collider_right = ColliderBuilder::cuboid(wall_thickness, screen_height() / PHYSICS_SCALE)
        .translation(vector![screen_width() / PHYSICS_SCALE, 0.0])
        .build();
    collider_set.insert(collider_right);
}

fn add_ball(
    x: f32,
    y: f32,
    rigid_body_set: &mut RigidBodySet,
    collider_set: &mut ColliderSet,
) -> Ball {
    let position = vector![x / PHYSICS_SCALE, y / -PHYSICS_SCALE];

    let rigid_body = RigidBodyBuilder::dynamic().translation(position).build();
    let collider = ColliderBuilder::ball(0.6).restitution(0.7).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, rigid_body_set);

    Ball {
        radius: 0.6,
        position,
        handle: ball_body_handle,
    }
}
