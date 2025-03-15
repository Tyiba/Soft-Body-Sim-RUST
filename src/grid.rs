
//use std::sync::{Arc, RwLock};
use rand::Rng;
use rayon::prelude::*;

// Constants outlined in the specification:
const MASS: f32 = 0.01;
const GRAVITY: f32 = -9.81;

const SPRING_RELAX_DISTANCE: f32 = 1.0;
const SPRING_COEFFICIENT: f32 = 10.0;
const DAMPING_COEFFICIENT: f32 = 0.03;
const EXTERNAL_MAGNITUDE: f32 = 0.2;


#[derive(Copy, Clone)]
pub struct Vertex {
    pub position: (f32, f32),
}

pub struct Grid {
    pub width: usize,
    pub height: usize,
    pub positions: Vec<(f32, f32)>,
    pub velocities: Vec<(f32, f32)>,
    pub fixed: Vec<bool>,
    pub neighbours: Vec<Vec<usize>>,
}

impl Grid {
    pub fn new(width: usize, height: usize) -> Grid {
        let size = width * height;
        let mut positions = Vec::with_capacity(size);
        let mut velocities = Vec::with_capacity(size);
        let mut fixed = Vec::with_capacity(size);
        let y_offset = 10.0;
        for x in 0..width {
            for y in 0..height {
                positions.push(
                               ((width  / 2) as f32 * -1.0 + x as f32, 
                                y_offset + (height / 2) as f32 * -1.0 + y as f32)
                              );
                velocities.push((0f32, 0f32));
                fixed.push(false);
            }
        }

        Grid {
            width,
            height,
            positions,
            velocities,
            fixed,
            neighbours: vec![vec![]; size],
        }
    }

    pub fn create_grid(&self) -> Vec<Vertex> {
        let mut lines = vec![];
        for x in 0..(self.width - 1) {
            for y in 0..(self.height - 1) {
                lines.push(Vertex {
                    position: self.positions[self.get_index(x, y)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x, y + 1)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x, y + 1)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x + 1, y + 1)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x + 1, y + 1)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x + 1, y)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x + 1, y)],
                });
                lines.push(Vertex {
                    position: self.positions[self.get_index(x, y)],
                });
            }
        }
        lines
    }

    pub fn get_index(&self, n: usize, m: usize) -> usize {
        n * self.height + m
    }

    pub fn get_neighbors(&mut self) {
        for x in 0..self.width {
            for y in 0..self.height {
                let mut neighbors = Vec::new();
                if x != (self.width - 1) {
                    neighbors.push(self.get_index(x + 1, y));
                }
                if x != 0 {
                    neighbors.push(self.get_index(x - 1, y));
                }
                if y != (self.height - 1) {
                    neighbors.push(self.get_index(x, y + 1));
                }
                if y != 0 {
                    neighbors.push(self.get_index(x, y - 1));
                }
                let index = self.get_index(x, y);
                self.neighbours[index] = neighbors;
            }
        }
    }

    pub fn calculate_forces(&mut self, delta_t: f32, externalbool: bool) {
        let positions = &self.positions;
        let velocities = &mut self.velocities;
        let fixed = &self.fixed;
        let neighbours = &self.neighbours;

        let new_positions: Vec<(f32, f32)> = positions
            .par_iter()
            .enumerate()
            .map(|(index, &position)| {
                if fixed[index] {
                    return position;
                }

                let mut total_force = (0.0, 0.0);
                let current_velocity = velocities[index];

                for &neighbor_index in &neighbours[index] {
                    let neighbor_position = positions[neighbor_index];
                    let displacement_x = neighbor_position.0 - position.0;
                    let displacement_y = neighbor_position.1 - position.1;
                    let distance = (displacement_x.powf(2.0) + displacement_y.powf(2.0)).sqrt();
                    let magnitude = SPRING_COEFFICIENT * (distance - SPRING_RELAX_DISTANCE);

                    let spring_force_x = magnitude * displacement_x / distance;
                    let spring_force_y = magnitude * displacement_y / distance;
                    total_force.0 += spring_force_x;
                    total_force.1 += spring_force_y;
                }

                let damper_force_x = -current_velocity.0 * DAMPING_COEFFICIENT;
                let damper_force_y = -current_velocity.1 * DAMPING_COEFFICIENT;
                total_force.0 += damper_force_x;
                total_force.1 += damper_force_y;

                if externalbool {
                    let mut random = rand::thread_rng();
                    let random_force_x = random.gen_range(-1.0..1.0) * EXTERNAL_MAGNITUDE;
                    let random_force_y = random.gen_range(-1.0..1.0) * EXTERNAL_MAGNITUDE;
                    total_force.0 += random_force_x;
                    total_force.1 += random_force_y;
                }  
                let acceleration_x = total_force.0 / MASS;
                let acceleration_y = total_force.1 / MASS;

                let new_position_x = position.0 + current_velocity.0 * delta_t + 0.5 * acceleration_x * delta_t.powf(2.0);
                let new_position_y = position.1 + current_velocity.1 * delta_t + 0.5 * acceleration_y * delta_t.powf(2.0);

                (new_position_x, new_position_y)
            })
            .collect();

        let new_velocities: Vec<(f32, f32)> = new_positions
            .par_iter()
            .enumerate()
            .map(|(index, &new_position)| {
                if fixed[index] {
                    return velocities[index];
                }
                let old_position = positions[index];
                let new_velocity_x = (new_position.0 - old_position.0) / delta_t;
                let new_velocity_y = (new_position.1 - old_position.1) / delta_t;
                (new_velocity_x, new_velocity_y)
            })
            .collect();

        self.positions = new_positions;
        self.velocities = new_velocities;
    }

    pub fn calculate_forces_with_gravity(&mut self, delta_t: f32, externalbool: bool) {
        let positions = &self.positions;
        let velocities = &mut self.velocities;
        let fixed = &self.fixed;
        let neighbours = &self.neighbours;

        let new_positions: Vec<(f32, f32)> = positions
            .par_iter()
            .enumerate()
            .map(|(index, &position)| {
                if fixed[index] {
                    return position;
                }

                let mut total_force = (0.0, 0.0);
                let current_velocity = velocities[index];

                for &neighbor_index in &neighbours[index] {
                    let neighbor_position = positions[neighbor_index];
                    let displacement_x = neighbor_position.0 - position.0;
                    let displacement_y = neighbor_position.1 - position.1;
                    let distance = (displacement_x.powf(2.0) + displacement_y.powf(2.0)).sqrt();
                    let magnitude = SPRING_COEFFICIENT * (distance - SPRING_RELAX_DISTANCE);

                    let spring_force_x = magnitude * displacement_x / distance;
                    let spring_force_y = magnitude * displacement_y / distance;
                    total_force.0 += spring_force_x;
                    total_force.1 += spring_force_y;
                }

                let damper_force_x = -current_velocity.0 * DAMPING_COEFFICIENT;
                let damper_force_y = -current_velocity.1 * DAMPING_COEFFICIENT;
                total_force.0 += damper_force_x;
                total_force.1 += damper_force_y;

                let gravity_force_y = GRAVITY * MASS;
                total_force.1 += gravity_force_y;

                if externalbool {
                    let mut random = rand::thread_rng();
                    let random_force_x = random.gen_range(-1.0..1.0) * EXTERNAL_MAGNITUDE;
                    let random_force_y = random.gen_range(-1.0..1.0) * EXTERNAL_MAGNITUDE;
                    total_force.0 += random_force_x;
                    total_force.1 += random_force_y;
                }  

                let acceleration_x = total_force.0 / MASS;
                let acceleration_y = total_force.1 / MASS;

                let new_position_x = position.0 + current_velocity.0 * delta_t + 0.5 * acceleration_x * delta_t.powf(2.0);
                let new_position_y = position.1 + current_velocity.1 * delta_t + 0.5 * acceleration_y * delta_t.powf(2.0);

                (new_position_x, new_position_y)
            })
            .collect();

        let new_velocities: Vec<(f32, f32)> = new_positions
            .par_iter()
            .enumerate()
            .map(|(index, &new_position)| {
                if fixed[index] {
                    return velocities[index];
                }
                let old_position = positions[index];
                let new_velocity_x = (new_position.0 - old_position.0) / delta_t;
                let new_velocity_y = (new_position.1 - old_position.1) / delta_t;
                (new_velocity_x, new_velocity_y)
            })
            .collect();

        self.positions = new_positions;
        self.velocities = new_velocities;
    }
}
