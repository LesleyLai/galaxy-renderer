mod galaxy;

use std::iter;

use winit::{
    event::*,
    event_loop::{ControlFlow, EventLoop},
    window::{Window, WindowBuilder},
};

use std::f32::consts::{PI, FRAC_PI_4};

use wgpu::util::DeviceExt;
use winit::dpi::PhysicalSize;
use crate::galaxy::Galaxy;


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
    color: [f32; 3],
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
    indices.extend(((index_offset)..(index_offset + vertices_count - 1)).flat_map(|n| n..(n + 2)));
    indices.push(index_offset + vertices_count - 1);
    indices.push(index_offset);
}

struct Camera {
    eye: cgmath::Point3<f32>,
    target: cgmath::Point3<f32>,
    up: cgmath::Vector3<f32>,
    aspect: f32,
    fovy: f32,
    znear: f32,
    zfar: f32,
}

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 1.0,
);

impl Camera {
    fn build_view_projection_matrix(&self) -> cgmath::Matrix4<f32> {
        let view = cgmath::Matrix4::look_at_rh(self.eye, self.target, self.up);
        let proj = cgmath::perspective(cgmath::Deg(self.fovy), self.aspect, self.znear, self.zfar);
        return OPENGL_TO_WGPU_MATRIX * proj * view;
    }
}

#[repr(C)]
// This is so we can store this in a buffer
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
struct CameraUniform {
    // We can't use cgmath with bytemuck directly so we'll have
    // to convert the Matrix4 into a 4x4 f32 array
    view_proj: [[f32; 4]; 4],
}

impl CameraUniform {
    fn new() -> Self {
        use cgmath::SquareMatrix;
        Self {
            view_proj: cgmath::Matrix4::identity().into(),
        }
    }

    fn update_view_proj(&mut self, camera: &Camera) {
        self.view_proj = camera.build_view_projection_matrix().into();
    }
}

struct CameraController {
    speed: f32,
    is_forward_pressed: bool,
    is_backward_pressed: bool,
    is_left_pressed: bool,
    is_right_pressed: bool,
}

impl CameraController {
    fn new(speed: f32) -> Self {
        Self {
            speed,
            is_forward_pressed: false,
            is_backward_pressed: false,
            is_left_pressed: false,
            is_right_pressed: false,
        }
    }

    fn process_events(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::KeyboardInput {
                input: KeyboardInput {
                    state,
                    virtual_keycode: Some(keycode),
                    ..
                },
                ..
            } => {
                let is_pressed = *state == ElementState::Pressed;
                match keycode {
                    VirtualKeyCode::W | VirtualKeyCode::Up => {
                        self.is_forward_pressed = is_pressed;
                        true
                    }
                    VirtualKeyCode::A | VirtualKeyCode::Left => {
                        self.is_left_pressed = is_pressed;
                        true
                    }
                    VirtualKeyCode::S | VirtualKeyCode::Down => {
                        self.is_backward_pressed = is_pressed;
                        true
                    }
                    VirtualKeyCode::D | VirtualKeyCode::Right => {
                        self.is_right_pressed = is_pressed;
                        true
                    }
                    _ => false,
                }
            }
            _ => false,
        }
    }

    fn update_camera(&self, camera: &mut Camera) {
        use cgmath::InnerSpace;
        let forward = camera.target - camera.eye;
        let forward_norm = forward.normalize();
        let forward_mag = forward.magnitude();

        // Prevents glitching when camera gets too close to the
        // center of the scene.
        if self.is_forward_pressed && forward_mag > self.speed {
            camera.eye += forward_norm * self.speed;
        }
        if self.is_backward_pressed {
            camera.eye -= forward_norm * self.speed;
        }

        let right = forward_norm.cross(camera.up);
        let up = forward_norm.cross(right);


        // Redo radius calc in case the fowrard/backward is pressed.
        let forward = camera.target - camera.eye;
        let forward_mag = forward.magnitude();

        if self.is_right_pressed {
            // Rescale the distance between the target and eye so
            // that it doesn't change. The eye therefore still
            // lies on the circle made by the target and eye.
            camera.eye = camera.target - (forward + up * self.speed).normalize() * forward_mag;
        }
        if self.is_left_pressed {
            camera.eye = camera.target - (forward - up * self.speed).normalize() * forward_mag;
        }
    }
}

fn color_from_temperature(temperature: f32) -> [f32; 3]
{
    let color_table: [[f32; 3]; 200] = [
        [1.0, -0.00987248, -0.0166818],
        [1.0, 0.000671682, -0.0173831],
        [1.0, 0.0113477, -0.0179839],
        [1.0, 0.0221357, -0.0184684],
        [1.0, 0.0330177, -0.0188214],
        [1.0, 0.0439771, -0.0190283],
        [1.0, 0.0549989, -0.0190754],
        [1.0, 0.0660696, -0.0189496],
        [1.0, 0.0771766, -0.0186391],
        [1.0, 0.0883086, -0.0181329],
        [1.0, 0.0994553, -0.017421],
        [1.0, 0.110607, -0.0164945],
        [1.0, 0.121756, -0.0153455],
        [1.0, 0.132894, -0.0139671],
        [1.0, 0.144013, -0.0123534],
        [1.0, 0.155107, -0.0104993],
        [1.0, 0.166171, -0.0084008],
        [1.0, 0.177198, -0.00605465],
        [1.0, 0.188184, -0.00345843],
        [1.0, 0.199125, -0.000610485],
        [1.0, 0.210015, 0.00249014],
        [1.0, 0.220853, 0.00584373],
        [1.0, 0.231633, 0.00944995],
        [1.0, 0.242353, 0.0133079],
        [1.0, 0.25301, 0.0174162],
        [1.0, 0.263601, 0.021773],
        [1.0, 0.274125, 0.0263759],
        [1.0, 0.284579, 0.0312223],
        [1.0, 0.294962, 0.0363091],
        [1.0, 0.305271, 0.0416328],
        [1.0, 0.315505, 0.0471899],
        [1.0, 0.325662, 0.0529765],
        [1.0, 0.335742, 0.0589884],
        [1.0, 0.345744, 0.0652213],
        [1.0, 0.355666, 0.0716707],
        [1.0, 0.365508, 0.078332],
        [1.0, 0.375268, 0.0852003],
        [1.0, 0.384948, 0.0922709],
        [1.0, 0.394544, 0.0995389],
        [1.0, 0.404059, 0.106999],
        [1.0, 0.41349, 0.114646],
        [1.0, 0.422838, 0.122476],
        [1.0, 0.432103, 0.130482],
        [1.0, 0.441284, 0.138661],
        [1.0, 0.450381, 0.147005],
        [1.0, 0.459395, 0.155512],
        [1.0, 0.468325, 0.164175],
        [1.0, 0.477172, 0.172989],
        [1.0, 0.485935, 0.181949],
        [1.0, 0.494614, 0.19105],
        [1.0, 0.503211, 0.200288],
        [1.0, 0.511724, 0.209657],
        [1.0, 0.520155, 0.219152],
        [1.0, 0.528504, 0.228769],
        [1.0, 0.536771, 0.238502],
        [1.0, 0.544955, 0.248347],
        [1.0, 0.553059, 0.2583],
        [1.0, 0.561082, 0.268356],
        [1.0, 0.569024, 0.27851],
        [1.0, 0.576886, 0.288758],
        [1.0, 0.584668, 0.299095],
        [1.0, 0.592372, 0.309518],
        [1.0, 0.599996, 0.320022],
        [1.0, 0.607543, 0.330603],
        [1.0, 0.615012, 0.341257],
        [1.0, 0.622403, 0.35198],
        [1.0, 0.629719, 0.362768],
        [1.0, 0.636958, 0.373617],
        [1.0, 0.644122, 0.384524],
        [1.0, 0.65121, 0.395486],
        [1.0, 0.658225, 0.406497],
        [1.0, 0.665166, 0.417556],
        [1.0, 0.672034, 0.428659],
        [1.0, 0.678829, 0.439802],
        [1.0, 0.685552, 0.450982],
        [1.0, 0.692204, 0.462196],
        [1.0, 0.698786, 0.473441],
        [1.0, 0.705297, 0.484714],
        [1.0, 0.711739, 0.496013],
        [1.0, 0.718112, 0.507333],
        [1.0, 0.724417, 0.518673],
        [1.0, 0.730654, 0.53003],
        [1.0, 0.736825, 0.541402],
        [1.0, 0.742929, 0.552785],
        [1.0, 0.748968, 0.564177],
        [1.0, 0.754942, 0.575576],
        [1.0, 0.760851, 0.586979],
        [1.0, 0.766696, 0.598385],
        [1.0, 0.772479, 0.609791],
        [1.0, 0.778199, 0.621195],
        [1.0, 0.783858, 0.632595],
        [1.0, 0.789455, 0.643989],
        [1.0, 0.794991, 0.655375],
        [1.0, 0.800468, 0.666751],
        [1.0, 0.805886, 0.678116],
        [1.0, 0.811245, 0.689467],
        [1.0, 0.816546, 0.700803],
        [1.0, 0.82179, 0.712122],
        [1.0, 0.826976, 0.723423],
        [1.0, 0.832107, 0.734704],
        [1.0, 0.837183, 0.745964],
        [1.0, 0.842203, 0.757201],
        [1.0, 0.847169, 0.768414],
        [1.0, 0.852082, 0.779601],
        [1.0, 0.856941, 0.790762],
        [1.0, 0.861748, 0.801895],
        [1.0, 0.866503, 0.812999],
        [1.0, 0.871207, 0.824073],
        [1.0, 0.87586, 0.835115],
        [1.0, 0.880463, 0.846125],
        [1.0, 0.885017, 0.857102],
        [1.0, 0.889521, 0.868044],
        [1.0, 0.893977, 0.878951],
        [1.0, 0.898386, 0.889822],
        [1.0, 0.902747, 0.900657],
        [1.0, 0.907061, 0.911453],
        [1.0, 0.91133, 0.922211],
        [1.0, 0.915552, 0.932929],
        [1.0, 0.91973, 0.943608],
        [1.0, 0.923863, 0.954246],
        [1.0, 0.927952, 0.964842],
        [1.0, 0.931998, 0.975397],
        [1.0, 0.936001, 0.985909],
        [1.0, 0.939961, 0.996379],
        [0.993241, 0.9375, 1.0],
        [0.983104, 0.931743, 1.0],
        [0.973213, 0.926103, 1.0],
        [0.963562, 0.920576, 1.0],
        [0.954141, 0.915159, 1.0],
        [0.944943, 0.909849, 1.0],
        [0.935961, 0.904643, 1.0],
        [0.927189, 0.899538, 1.0],
        [0.918618, 0.894531, 1.0],
        [0.910244, 0.88962, 1.0],
        [0.902059, 0.884801, 1.0],
        [0.894058, 0.880074, 1.0],
        [0.886236, 0.875434, 1.0],
        [0.878586, 0.87088, 1.0],
        [0.871103, 0.86641, 1.0],
        [0.863783, 0.862021, 1.0],
        [0.856621, 0.857712, 1.0],
        [0.849611, 0.853479, 1.0],
        [0.84275, 0.849322, 1.0],
        [0.836033, 0.845239, 1.0],
        [0.829456, 0.841227, 1.0],
        [0.823014, 0.837285, 1.0],
        [0.816705, 0.83341, 1.0],
        [0.810524, 0.829602, 1.0],
        [0.804468, 0.825859, 1.0],
        [0.798532, 0.82218, 1.0],
        [0.792715, 0.818562, 1.0],
        [0.787012, 0.815004, 1.0],
        [0.781421, 0.811505, 1.0],
        [0.775939, 0.808063, 1.0],
        [0.770561, 0.804678, 1.0],
        [0.765287, 0.801348, 1.0],
        [0.760112, 0.798071, 1.0],
        [0.755035, 0.794846, 1.0],
        [0.750053, 0.791672, 1.0],
        [0.745164, 0.788549, 1.0],
        [0.740364, 0.785474, 1.0],
        [0.735652, 0.782448, 1.0],
        [0.731026, 0.779468, 1.0],
        [0.726482, 0.776534, 1.0],
        [0.722021, 0.773644, 1.0],
        [0.717638, 0.770798, 1.0],
        [0.713333, 0.767996, 1.0],
        [0.709103, 0.765235, 1.0],
        [0.704947, 0.762515, 1.0],
        [0.700862, 0.759835, 1.0],
        [0.696848, 0.757195, 1.0],
        [0.692902, 0.754593, 1.0],
        [0.689023, 0.752029, 1.0],
        [0.685208, 0.749502, 1.0],
        [0.681458, 0.747011, 1.0],
        [0.67777, 0.744555, 1.0],
        [0.674143, 0.742134, 1.0],
        [0.670574, 0.739747, 1.0],
        [0.667064, 0.737394, 1.0],
        [0.663611, 0.735073, 1.0],
        [0.660213, 0.732785, 1.0],
        [0.656869, 0.730528, 1.0],
        [0.653579, 0.728301, 1.0],
        [0.65034, 0.726105, 1.0],
        [0.647151, 0.723939, 1.0],
        [0.644013, 0.721801, 1.0],
        [0.640922, 0.719692, 1.0],
        [0.637879, 0.717611, 1.0],
        [0.634883, 0.715558, 1.0],
        [0.631932, 0.713531, 1.0],
        [0.629025, 0.711531, 1.0],
        [0.626162, 0.709557, 1.0],
        [0.623342, 0.707609, 1.0],
        [0.620563, 0.705685, 1.0],
        [0.617825, 0.703786, 1.0],
        [0.615127, 0.701911, 1.0],
        [0.612469, 0.70006, 1.0],
        [0.609848, 0.698231, 1.0],
        [0.607266, 0.696426, 1.0],
        [0.60472, 0.694643, 1.0]
    ];

    let min_temperature = 1000;
    let max_temperature = 10000;
    let col_count = color_table.len() as i32;

    let idx: i32 = ((temperature - min_temperature as f32) / (max_temperature - min_temperature) as f32 * col_count as f32) as i32;
    let idx = idx.clamp(0, col_count - 1) as usize;
    color_table[idx]
}

struct State {
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    size: winit::dpi::PhysicalSize<u32>,
    star_render_pipeline: wgpu::RenderPipeline,
    star_vertex_buffer: wgpu::Buffer,
    star_vertex_count: u32,

    curve_render_pipeline: wgpu::RenderPipeline,
    curve_vertex_buffer: wgpu::Buffer,
    curve_index_buffer: wgpu::Buffer,
    curve_index_count: u32,

    camera: Camera,
    camera_uniform: CameraUniform,
    camera_buffer: wgpu::Buffer,
    camera_bind_group: wgpu::BindGroup,
    camera_controller: CameraController,

    show_guidelines: bool,
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
            format: surface.get_supported_formats(&adapter)[0],
            width: size.width,
            height: size.height,
            present_mode: wgpu::PresentMode::Fifo,
        };
        surface.configure(&device, &config);

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
        });

        let camera_bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }
            ],
            label: Some("camera_bind_group_layout"),
        });

        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Render Pipeline Layout"),
                bind_group_layouts: &[&camera_bind_group_layout],
                push_constant_ranges: &[],
            });

        let star_render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Star Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::PointList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        let curve_render_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Curve Render Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_main",
                buffers: &[Vertex::desc()],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_main",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                polygon_mode: wgpu::PolygonMode::Fill,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState {
                count: 1,
                mask: !0,
                alpha_to_coverage_enabled: false,
            },
            multiview: None,
        });

        let galaxy = Galaxy {
            star_count: 50000,
            r_bulge: 0.2,
        };
        let stars = galaxy.generate_stars();

        let star_vertices = stars
            .iter()
            .map(|star| {
                let sin_theta = star.curve_offset.sin();
                let cos_theta = star.curve_offset.cos();

                // The rotation angle of vertex in the ellipse's coordinate
                let phi = star.start_position;
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();

                let a = star.x_radius;
                let b = star.y_radius;

                let x = a * cos_phi * cos_theta - b * sin_phi * sin_theta;
                let y = a * cos_phi * sin_theta + b * sin_phi * cos_theta;

                let color = color_from_temperature(star.temperature);

                Vertex {
                    position: [x, y, star.elevation],
                    color: color,
                }
            })
            .collect::<Vec<Vertex>>();

        let mut curve_vertices = vec![];
        let mut curve_indices = vec![];
        let mut append_ellipse_vertices =
            |color, create_info|
                append_ellipse_vertices(&mut curve_vertices, &mut curve_indices, color, create_info);

        // Guideline for density waves
        for i in 0..100 {
            let fi = i as f32;
            let x_radius = galaxy.r_bulge + 0.02 * fi;
            let y_radius = x_radius + 0.1;
            let curve_offset = x_radius * (2.0 * PI);
            append_ellipse_vertices(
                [0.2, 0.2, 0.2],
                EllipseCreateInfo {
                    a: x_radius,
                    b: y_radius,
                    angle: curve_offset,
                    vertices_count: 100,
                },
            );
        }

        // Guideline for Bulga
        append_ellipse_vertices(
            [0.0, 1.0, 0.0],
            EllipseCreateInfo {
                a: galaxy.r_bulge,
                b: galaxy.r_bulge,
                angle: 0.0,
                vertices_count: 64,
            },
        );

        // Guideline for Galaxy Radius
        append_ellipse_vertices(
            [0.0, 0.0, 1.0],
            EllipseCreateInfo {
                a: 1.0,
                b: 1.0,
                angle: 0.0,
                vertices_count: 128,
            },
        );

        // Guideline for Far-field Radius
        append_ellipse_vertices(
            [1.0, 0.0, 0.0],
            EllipseCreateInfo {
                a: 2.0,
                b: 2.0,
                angle: 0.0,
                vertices_count: 256,
            },
        );


        let star_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Star Vertex Buffer"),
            contents: bytemuck::cast_slice(star_vertices.as_slice()),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let curve_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Curve Vertex Buffer"),
            contents: bytemuck::cast_slice(curve_vertices.as_slice()),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let curve_index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Curve Indices Buffer"),
            contents: bytemuck::cast_slice(curve_indices.as_slice()),
            usage: wgpu::BufferUsages::INDEX,
        });

        let camera = Camera {
            eye: (0.0, 0.0, 50.0).into(),
            // have it look at the origin
            target: (0.0, 0.0, 0.0).into(),
            // which way is "up"
            up: cgmath::Vector3::unit_y(),
            aspect: config.width as f32 / config.height as f32,
            fovy: FRAC_PI_4,
            znear: 0.1,
            zfar: 1000.0,
        };

        let mut camera_uniform = CameraUniform::new();
        camera_uniform.update_view_proj(&camera);

        let camera_buffer = device.create_buffer_init(
            &wgpu::util::BufferInitDescriptor {
                label: Some("Camera Buffer"),
                contents: bytemuck::cast_slice(&[camera_uniform]),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            }
        );


        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            layout: &camera_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: camera_buffer.as_entire_binding(),
                }
            ],
            label: Some("camera_bind_group"),
        });

        let camera_controller = CameraController::new(0.5);

        Self {
            surface,
            device,
            queue,
            config,
            size,
            star_render_pipeline,
            star_vertex_buffer,
            star_vertex_count: star_vertices.len() as u32,
            curve_render_pipeline: curve_render_pipeline,
            curve_vertex_buffer,
            curve_index_buffer,
            curve_index_count: curve_indices.len() as u32,

            camera,
            camera_uniform,
            camera_buffer,
            camera_bind_group,
            camera_controller,

            show_guidelines: true,
        }
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        if new_size.width > 0 && new_size.height > 0 {
            self.size = new_size;
            self.config.width = new_size.width;
            self.config.height = new_size.height;
            self.surface.configure(&self.device, &self.config);

            self.camera.aspect = new_size.width as f32 / new_size.height as f32;
        }
    }

    fn input(&mut self, event: &WindowEvent) -> bool {
        match event {
            WindowEvent::KeyboardInput {
                input: KeyboardInput {
                    state,
                    virtual_keycode: Some(keycode),
                    ..
                },
                ..
            } => {
                if *state == ElementState::Released {
                    match keycode {
                        VirtualKeyCode::G => {
                            self.show_guidelines = !self.show_guidelines;
                        }
                        _ => ()
                    }
                }
            }
            _ => ()
        }

        self.camera_controller.process_events(event)
    }

    fn update(&mut self) {
        self.camera_controller.update_camera(&mut self.camera);
        self.camera_uniform.update_view_proj(&self.camera);
        self.queue.write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[self.camera_uniform]));
    }

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
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
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
                })],
                depth_stencil_attachment: None,
            });

            render_pass.set_bind_group(0, &self.camera_bind_group, &[]);
            render_pass.set_pipeline(&self.star_render_pipeline);
            render_pass.set_vertex_buffer(0, self.star_vertex_buffer.slice(..));
            render_pass.draw(0..self.star_vertex_count, 0..1);

            if self.show_guidelines {
                render_pass.set_pipeline(&self.curve_render_pipeline);
                render_pass.set_vertex_buffer(0, self.curve_vertex_buffer.slice(..));
                render_pass.set_index_buffer(self.curve_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
                render_pass.draw_indexed(0..self.curve_index_count, 0, 0..1);
            }
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
