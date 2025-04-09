struct CameraUniform {
    view_proj: mat4x4<f32>,
};

@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct Star {
    @location(0) position: vec4<f32>, // w is padding
    @location(1) color: vec4<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) uv: vec2<f32>,
};

struct Vertex {
  pos: vec2<f32>,
  uv: vec2<f32>
}

fn quad_vertex_pos_from_index(index: u32) -> Vertex
{
    var vertex: Vertex;
    switch (index) {
        case 0u {
            vertex.pos = vec2<f32>(-0.01, 0.01);
            vertex.uv = vec2<f32>(0.0, 1.0);
        }
        case 1u {
            vertex.pos = vec2<f32>(-0.01, -0.01);
            vertex.uv = vec2<f32>(0.0, 0.0);
        }
        case 2u {
            vertex.pos = vec2<f32>(0.01, 0.01);
            vertex.uv = vec2<f32>(1.0, 1.0);
        }
        case 3u {
            vertex.pos = vec2<f32>(0.01, -0.01);
            vertex.uv = vec2<f32>(1.0, 0.0);
        }
        default { // should never happen
        }
    }
    return vertex;
}

@vertex
fn vs_main(
    @builtin(vertex_index) in_vertex_index: u32,
    @builtin(instance_index) in_instance_index: u32,
    star: Star,
) -> VertexOutput {
    var out: VertexOutput;

    out.clip_position = camera.view_proj * vec4<f32>(star.position.xyz, 1.0);

    // Offset clip position to the four vertices of the quad
    out.clip_position = out.clip_position / out.clip_position.w;

    let quad_offset = quad_vertex_pos_from_index(in_vertex_index);
    out.clip_position += vec4<f32>(quad_offset.pos * 0.3, 0.0, 0.0);

    out.color = vec4<f32>(star.color.xyz, 1.0);
    out.uv = quad_offset.uv;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let uv = in.uv;

    let d = abs(uv - 0.5) * 2.0;
    let d2 = dot(d, d);

    if (d2 > 1.0) {
      discard;
    }
    let color = in.color.xyz;
    let alpha = (1.0 - sqrt(d2));
    return vec4<f32>(color, alpha);
}