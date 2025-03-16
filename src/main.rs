#[macro_use]
extern crate glium;
extern crate winit;
extern crate num_cpus;

use std::sync::{Arc, RwLock};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread; 
use std::time::{Duration , Instant};
use glium::Surface;
use rayon::ThreadPoolBuilder;

use crate::grid::Grid;
use crate::grid::Vertex;

mod grid;

const DELTA_TIME: f32 = 0.01;
const LOG_DURATION: u64 = 10; // In seconds
static GRAVITY_ACTIVE: AtomicBool = 
AtomicBool::new(true);
static EXTERNAL_MAGNITUDE: AtomicBool = 
AtomicBool::new(false);
const HEIGHT: usize = 30;
const WIDTH: usize = 30;

fn run_threaded(grid: Arc<RwLock<Grid>>, thread_count: usize) ->  std::thread::JoinHandle<()> {
    
    ThreadPoolBuilder::new().num_threads(thread_count).build_global().unwrap();

    let handle = thread::spawn(move || {
        let mut total_duration = Duration::new(0, 0);
        let mut iterations = 0;
        let start_time = Instant::now();

        while start_time.elapsed().as_secs() < LOG_DURATION {
            let start = Instant::now();
            {
                let mut grid = grid.write().unwrap();
                let current = EXTERNAL_MAGNITUDE.load(Ordering::Relaxed);
                if HEIGHT < 100
                {
                    for _ in 0..20
                    {
                        
                        if GRAVITY_ACTIVE.load(Ordering::Relaxed) {
                            grid.calculate_forces_with_gravity(DELTA_TIME, current);
                        } else {
                            grid.calculate_forces(DELTA_TIME, current);
                        }
                    }
                }
                else
                {
                    if GRAVITY_ACTIVE.load(Ordering::Relaxed) {
                        grid.calculate_forces_with_gravity(DELTA_TIME, current);
                    } else {
                        grid.calculate_forces(DELTA_TIME, current);
                    }
                }
            }
            let duration = start.elapsed();
            total_duration += duration;
            iterations += 1;

            //println!("Time taken for update with {} threads: {:?}", thread_count, duration);
            thread::sleep(Duration::from_secs_f32(DELTA_TIME));
        }

        let average_duration = total_duration / iterations;
        println!("Average time taken for update with {} threads over {} seconds: {:?}", 
        thread_count, LOG_DURATION, average_duration);
    });
    handle
}

fn render(grid: Arc<RwLock<Grid>>) {
    //rendering taken from triangles lab
    let event_loop = winit::event_loop::EventLoopBuilder::new().build().expect("event loop building");
    let (_window, display) = glium::backend::glutin::SimpleWindowBuilder::new().with_title("600086-Lab-I Soft body physics").with_inner_size(800, 800).build(&event_loop);

    implement_vertex!(Vertex, position);
    let indices = glium::index::NoIndices(glium::index::PrimitiveType::LinesList);

    pub const VERT_SHADER: &str = r#"
    #version 140

    in vec2 position;

    void main() {
        gl_Position = vec4(position, 0.0, 25.0);
    }
    "#;

    pub const FRAG_SHADER: &str = r#"
    #version 140

    out vec4 color;

    void main() {
        color = vec4(1.0, 1.0, 1.0, 1.0);
    }
    "#;

    let program = glium::Program::from_source(&display, VERT_SHADER, FRAG_SHADER, None).unwrap();

    let _ = event_loop.run(move |event, window_target| {
        match event {
            winit::event::Event::WindowEvent { event, .. } => match event {
                winit::event::WindowEvent::CloseRequested => window_target.exit(),
                winit::event::WindowEvent::Resized(window_size) => {
                    display.resize(window_size.into());
                },
                winit::event::WindowEvent::KeyboardInput { event, .. } => {
                    if event.state == winit::event::ElementState::Pressed && !event.repeat {
                        match event.logical_key {
                            winit::keyboard::Key::Character(c) if c == "G" || c == "g" => {
                                let current_state = GRAVITY_ACTIVE.load(Ordering::Relaxed);
                                    GRAVITY_ACTIVE.store(!current_state, Ordering::Relaxed);
                                    println!("Gravity toggled: {}", !current_state);
                            }
                            winit::keyboard::Key::Character(c) if c == "W" || c == "w" => {
                                let current_state = EXTERNAL_MAGNITUDE.load(Ordering::Relaxed);
                                    EXTERNAL_MAGNITUDE.store(!current_state, Ordering::Relaxed);
                                    println!("External toggled: {}", !current_state);
                            }
                            _ => (),
                        }
                    }
                    
                }
                winit::event::WindowEvent::RedrawRequested => {
                    let next_frame_time = std::time::Instant::now() + Duration::from_secs(DELTA_TIME as u64);
                    winit::event_loop::ControlFlow::WaitUntil(next_frame_time);

                    let vertex_buffer = glium::VertexBuffer::new(&display, &*grid.read().unwrap().create_grid()).unwrap();

                    let mut target = display.draw();
                    target.clear_color(0.0, 0.0, 0.0, 1.0);
                    target.draw(&vertex_buffer, &indices, &program, &glium::uniforms::EmptyUniforms, &Default::default()).unwrap();
                    target.finish().unwrap();
                },
                _ => (),
            },
            winit::event::Event::AboutToWait => {
                _window.request_redraw();
            },
            _ => (),
        };
    });
}

fn main() {
    let grid = Arc::new(RwLock::new(Grid::new(WIDTH, HEIGHT)));
    grid.write().unwrap().get_neighbors();

    let fixed_1 = grid.read().unwrap().get_index(0, HEIGHT - 1);
    let fixed_2 = grid.read().unwrap().get_index(WIDTH - 1,HEIGHT -1);
    
    {
        let mut grid_write = grid.write().unwrap();
        grid_write.fixed[fixed_1] = true;
        grid_write.fixed[fixed_2] = true;
    }

    let core_count = num_cpus::get() / 2;
    println!("CPU core count: {}", core_count);

    let thread_count = core_count;
    println!("Running simulation with {} threads", thread_count);

    let update_grid = grid.clone();
    let sim_handle = run_threaded(update_grid, thread_count);

    let enable_rendering = true; // Set this to false to disable rendering

    if enable_rendering {
        render(grid.clone());
    } else {
        // Join the simulation thread if rendering is disabled
        sim_handle.join().unwrap();
    }
    
}
