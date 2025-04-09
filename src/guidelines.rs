use std::f32::consts::PI;
use wgpu::util::DeviceExt;

use crate::galaxy::Galaxy;

#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 4],
    color: [f32; 4],
}

impl Vertex {
    const ATTRIBS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x4, 1 => Float32x4];

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
            position: [vx, vy, 0.0, 0.0],
            color: [color[0], color[1], color[2], 0.0],
        }
    }));

    // close the loop
    vertices.push(Vertex {
        position: [a * cos_theta, a * sin_theta, 0.0, 0.0],
        color: [color[0], color[1], color[2], 0.0],
    });

    // Indices in the form of 0 1 | 1 2 | 2 3 | 3 4 | 4 5 | 5 0
    indices.extend(((index_offset)..(index_offset + vertices_count - 1)).flat_map(|n| n..(n + 2)));
    indices.push(index_offset + vertices_count - 1);
    indices.push(index_offset);
}

pub struct GuidelineRenderer {
    curve_render_pipeline: wgpu::RenderPipeline,
    curve_vertex_buffer: wgpu::Buffer,
    curve_index_buffer: wgpu::Buffer,
    curve_index_count: u32,
}

impl GuidelineRenderer {
    pub fn new(
        device: &wgpu::Device,
        config: &wgpu::SurfaceConfiguration,
        galaxy: &Galaxy,
        camera_bind_group_layout: &wgpu::BindGroupLayout,
    ) -> Self {
        let render_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("Curve Pipeline Layout"),
                bind_group_layouts: &[camera_bind_group_layout],
                push_constant_ranges: &[],
            });

        let curve_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("Shader"),
            source: wgpu::ShaderSource::Wgsl(include_str!("curve.wgsl").into()),
        });

        let curve_render_pipeline =
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("Curve Render Pipeline"),
                layout: Some(&render_pipeline_layout),
                vertex: wgpu::VertexState {
                    module: &curve_shader,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[Vertex::desc()],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &curve_shader,
                    entry_point: Some("fs_main"),
                    compilation_options: Default::default(),
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
                    cull_mode: None,
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
                cache: None,
            });

        let mut curve_vertices = vec![];
        let mut curve_indices = vec![];
        let mut append_ellipse_vertices = |color, create_info| {
            append_ellipse_vertices(&mut curve_vertices, &mut curve_indices, color, create_info)
        };

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

        Self {
            curve_render_pipeline,
            curve_vertex_buffer,
            curve_index_buffer,
            curve_index_count: curve_indices.len() as u32,
        }
    }

    pub fn render(&self, render_pass: &mut wgpu::RenderPass) {
        render_pass.set_pipeline(&self.curve_render_pipeline);
        render_pass.set_vertex_buffer(0, self.curve_vertex_buffer.slice(..));
        render_pass.set_index_buffer(self.curve_index_buffer.slice(..), wgpu::IndexFormat::Uint16);
        render_pass.draw_indexed(0..self.curve_index_count, 0, 0..1);
    }
}
