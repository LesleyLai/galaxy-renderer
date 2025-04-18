use std::f32::consts::PI;

use rand::rng;
use rand_distr::{Distribution, Exp, Normal, Uniform};

pub struct Star {
    // Rotation of the star trajectory (angle in radius)
    pub curve_offset: f32,
    // Start position of the star (angle in radius)
    pub start_position: f32,
    pub x_radius: f32,
    pub y_radius: f32,
    pub elevation: f32,
    pub temperature: f32,
}

pub struct Galaxy {
    pub star_count: u32,
    pub r_bulge: f32,
}

impl Galaxy {
    pub fn generate_stars(&self) -> Vec<Star> {
        let mut stars = vec![];

        let start_position_distribution = Uniform::new(0.0, 2.0 * PI);

        let kappa = 5.0;
        let bulge_x_radius_distribution = Exp::new(0.25 * kappa).unwrap();
        let disk_x_radius_distribution = Exp::new(1.0 / 0.5).unwrap();

        let temperature_distribution = Normal::new(6000.0, 2000.0).unwrap();

        for _ in 0..self.star_count {
            let mut x_radius = bulge_x_radius_distribution.sample(&mut rng());
            if x_radius > self.r_bulge {
                x_radius = self.r_bulge + disk_x_radius_distribution.sample(&mut rng());
            }

            let start_position = start_position_distribution.unwrap().sample(&mut rng());
            let curve_offset = x_radius * (2.0 * PI);

            let mut y_radius = x_radius;
            if x_radius > self.r_bulge {
                y_radius += 0.1f32.min(x_radius - self.r_bulge);
            }

            let elevation_distribution = Normal::new(0.0, 0.02 * 0.2f32.powf(x_radius)).unwrap();

            let elevation = elevation_distribution.sample(&mut rng());

            let temperature = temperature_distribution.sample(&mut rng());

            stars.push(Star {
                curve_offset,
                start_position,
                x_radius,
                y_radius,
                elevation,
                temperature,
            })
        }

        stars
    }
}
