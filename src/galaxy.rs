use std::f32::consts::PI;

use rand::{thread_rng};
use rand_distr::{Distribution, Exp, Uniform};

pub struct Star {
    // Rotation of the star trajectory (angle in radius)
    pub curve_offset: f32,
    // Start position of the star (angle in radius)
    pub start_position: f32,
    pub x_radius: f32,
    pub y_radius: f32,
    pub elevation: f32,
}

pub fn generate_stars() -> Vec<Star> {
    let mut stars = vec![];

    let start_position_distribution = Uniform::new(0.0, 2.0 * PI);
    let x_radius_distribution = Exp::new(3.0).unwrap();

    for _ in 0..10000 {
        let x_radius = x_radius_distribution.sample(&mut thread_rng());
        let start_position = start_position_distribution.sample(&mut thread_rng());
        let curve_offset = x_radius * (2.0 * PI);
        let y_radius = x_radius + 0.1;
        let elevation = 0.0;

        stars.push(Star {
            curve_offset,
            start_position,
            x_radius,
            y_radius,
            elevation,
        })
    }

    stars
}