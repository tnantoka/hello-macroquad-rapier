 use macroquad::{
     color::{GRAY, YELLOW},
     shapes::draw_circle,
     window::{clear_background, next_frame, screen_width, screen_height},
 };
 use rapier2d::{
     dynamics::{
         CCDSolver, ImpulseJointSet, IntegrationParameters, IslandManager, MultibodyJointSet,
         RigidBodyBuilder, RigidBodySet,
     },
     geometry::{BroadPhaseMultiSap, ColliderBuilder, ColliderSet, NarrowPhase},
     na::{vector, Vector2},
     pipeline::{PhysicsPipeline, QueryPipeline},
     prelude::nalgebra,
 };

const PHYSICS_SCALE: f32 = 50.0;

 struct Ball {
     radius: f32,
     position: Vector2<f32>,
 }

#[macroquad::main("Hello")]
async fn main() {
    let mut ball = Ball {
        radius: 0.6,
        position: vector![screen_width() / (2.0 * PHYSICS_SCALE), screen_height() / (2.0 * PHYSICS_SCALE) ],
    };

    let mut rigid_body_set = RigidBodySet::new();
    let mut collider_set = ColliderSet::new();

    let collider_half_thickness = 0.05;
    let collider = ColliderBuilder::cuboid(100.0, collider_half_thickness)
        .translation(vector![
            0.0,
            (screen_height() / -PHYSICS_SCALE) - collider_half_thickness
        ])
        .build();
    collider_set.insert(collider);

    let rigid_body = RigidBodyBuilder::dynamic()
        .translation(ball.position)
        .build();
    let collider = ColliderBuilder::ball(0.6).restitution(0.7).build();
    let ball_body_handle = rigid_body_set.insert(rigid_body);
    collider_set.insert_with_parent(collider, ball_body_handle, &mut rigid_body_set);

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

    loop {
        clear_background(GRAY);

        draw_circle(
            PHYSICS_SCALE * ball.position.x,
            -PHYSICS_SCALE * ball.position.y,
            PHYSICS_SCALE * ball.radius,
            YELLOW,
        );

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

        let ball_body = &rigid_body_set[ball_body_handle];
        println!("Ball altitude: {}", ball_body.translation().y);

        ball.position = *ball_body.translation();

        next_frame().await
    }
}
