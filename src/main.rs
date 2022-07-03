mod galaxy;

use std::iter;

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use std::f32::consts::PI;

use wgpu::util::DeviceExt;
use winit::dpi::PhysicalSize;


#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 3],
}

impl Vertex {
    const ATTRIBS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    fn desc<'a>() -> wgpu::VertexBufferLayout<'a> {
        use std::mem;

        wgpu::VertexBufferLayout {
            array_stride: mem::size_of::<Self>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRIBS,
        }
    }
}


///
/// # Fields
///
/// * `a` - Half width of the ellipse
/// * `b` - Half height of the ellipse
/// * `angle` - The angle of the ellipse rotation around the z-axis (in radians)
struct EllipseCreateInfo {
    a: f32,
    b: f32,
    angle: f32,
    vertices_count: u16,
}

fn append_ellipse_vertices(
    vertices: &mut Vec<Vertex>,
    indices: &mut Vec<u16>,
    create_info: EllipseCreateInfo,
) {
    let EllipseCreateInfo {
        a,
        b,
        angle: theta,
        vertices_count,
    } = create_info;

    assert!(vertices_count >= 2); // Can't draw an ellipse without at least two vertices

    let sin_theta = theta.sin();
    let cos_theta = theta.cos();

    let color = [1.0, 1.0, 1.0];

    let index_offset = vertices.len() as u16;

    // Vertices on the ellipse
    vertices.extend((0..vertices_count).map(|i| {
        // The rotation angle of vertex in the ellipse's coordinate
        let phi = (2.0 * PI) * (i as f32 / vertices_count as f32);
        let [sin_phi, cos_phi] = [phi.sin(), phi.cos()];

        let vx = a * cos_phi * cos_theta - b * sin_phi * sin_theta;
        let vy = a * cos_phi * sin_theta + b * sin_phi * cos_theta;

        Vertex {
            position: [vx, vy, 0.0],
            color,
        }
    }));

    // close the loop
    vertices.push(Vertex {
        position: [a * cos_theta, a * sin_theta, 0.0],
        color,
    });

    // Indices in the form of 0 1 | 1 2 | 2 3 | 3 4 | 4 5 | 5 0
    indices.extend(((index_offset + 1)..(index_offset + vertices_count)).flat_map(|n| n..(n + 2)));
    indices.push(index_offset + vertices_count - 1);
    indices.push(index_offset);
}

struct State {
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    size: winit::dpi::PhysicalSize<u32>,
    render_pipeline: wgpu::RenderPipeline,
    vertex_buffer: wgpu::Buffer,
    vertex_count: u32,
    //index_buffer: wgpu::Buffer,
    //index_count: u32,
}

impl State {
    async fn new(window: &Window) -> Self {
        let size = window.inner_size();

        // The instance is a handle to our GPU
        // BackendBit::PRIMARY => Vulkan + Metal + DX12 + Browser WebGPU
        let instance = wgpu::Instance::new(wgpu::Backends::all());
        let surface = unsafe { instance.create_surface(window) };
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .unwrap();

        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: None,
                    features: wgpu::Features::empty(),
                    limits: wgpu::Limits::default(),
                },
                // Some(&std::path::Path::new("trace")), // Trace path
                None,
            )
            .await
            .unwrap();

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface.get_preferred_format(&adapter).unwrap(),
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Fifo,
        };
        surface.configure(&device, &config);

        let shader = device.create_shader_module(&wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[],
                push_constant_ranges: &[],
            });

        let render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                }],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::PointList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                clamp_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
        });

        let stars = galaxy::generate_stars();

        let vertices = stars
            .iter()
            .map(|star| {
                let cos_theta = star.curve_offset.cos();
                let sin_theta = star.curve_offset.sin();

                // The rotation angle of vertex in the ellipse's coordinate
                let phi = star.start_position;
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();

                let a = star.x_radius;
                let b = star.y_radius;

                let x = a * cos_phi * cos_theta - b * sin_phi * sin_theta;
                let y = a * cos_phi * sin_theta + b * sin_phi * cos_theta;

                Vertex {
                    position: [x, y, star.elevation],
                    color: [1.0, 1.0, 1.0],
                }
            })
            .collect::<Vec<Vertex>>();

        // let mut indices = vec![];
        //
        // for i in 0..25 {
        //     let fi = i as f32;
        //     append_ellipse_vertices(
        //         &mut vertices,
        //         &mut indices,
        //         EllipseCreateInfo {
        //             a: 0.2 + 0.03 * fi,
        //             b: 0.1 + 0.03 * fi,
        //             angle: (PI / 15.) * fi,
        //             vertices_count: 100,
        //         },
        //     );
        // }

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Vertex Buffer"),
            contents: bytemuck::cast_slice(vertices.as_slice()),
            usage: wgpu::BufferUsages::VERTEX,
        });

        // let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        //     label: Some("Indices Buffer"),
        //     contents: bytemuck::cast_slice(indices.as_slice()),
        //     usage: wgpu::BufferUsages::INDEX,
        // });

        Self {
            surface,
            device,
            queue,
            config,
            size,
            render_pipeline,
            vertex_buffer,
            vertex_count: vertices.len() as u32,
        }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);
        }
    }

    #[allow(unused_variables)]
    fn input(&mut self, event: &WindowEvent) -> bool {
        false
    }

    fn update(&mut self) {}

    fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let output = self.surface.get_current_texture()?;
        let view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Render Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Render Pass"),
                color_attachments: &[wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.0,
                            g: 0.0,
                            b: 0.0,
                            a: 1.0,
                        }),
                        store: true,
                    },
                }],
                depth_stencil_attachment: None,
            });

            render_pass.set_pipeline(&self.render_pipeline);
            render_pass.set_vertex_buffer(0, self.vertex_buffer.slice(..));
            //render_pass.set_index_buffer(self.index_buffer.slice(..), wgpu::IndexFormat::Uint16);
            render_pass.draw(0..self.vertex_count, 0..1);
        }

        self.queue.submit(iter::once(encoder.finish()));
        output.present();

        Ok(())
    }
}

fn main() {
    env_logger::init();
    let event_loop = EventLoop::new();
    let window = WindowBuilder::new()
        .with_title("Galaxy Renderer")
        .with_inner_size(PhysicalSize {
            width: 800,
            height: 800,
        })
        .build(&event_loop)
        .unwrap();

    // State::new uses async code, so we're going to wait for it to finish
    let mut state: State = pollster::block_on(State::new(&window));

    event_loop.run(move |event, _, control_flow| {
        match event {
            Event::WindowEvent {
                ref event,
                window_id,
            } if window_id == window.id() => process_window_event(&mut state, control_flow, event),
            Event::RedrawRequested(_) => {
                state.update();
                match state.render() {
                    Ok(_) => {}
                    // Reconfigure the surface if lost
                    Err(wgpu::SurfaceError::Lost) => state.resize(state.size),
                    // The system is out of memory, we should probably quit
                    Err(wgpu::SurfaceError::OutOfMemory) => *control_flow = ControlFlow::Exit,
                    // All other errors (Outdated, Timeout) should be resolved by the next frame
                    Err(e) => eprintln!("{:?}", e),
                }
            }
            Event::RedrawEventsCleared => {
                // RedrawRequested will only trigger once, unless we manually
                // request it.
                window.request_redraw();
            }
            _ => {}
        }
    });
}

fn process_window_event(state: &mut State, control_flow: &mut ControlFlow, event: &WindowEvent) {
    if !state.input(event) {
        match event {
            WindowEvent::CloseRequested
            | WindowEvent::KeyboardInput {
                input:
                KeyboardInput {
                    state: ElementState::Pressed,
                    virtual_keycode: Some(VirtualKeyCode::Escape),
                    ..
                },
                ..
            } => *control_flow = ControlFlow::Exit,
            WindowEvent::Resized(physical_size) => {
                state.resize(*physical_size);
            }
            WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                // new_inner_size is &&mut so w have to dereference it twice
                state.resize(**new_inner_size);
            }
            _ => {}
        }
    }
}
