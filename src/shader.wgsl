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
};

fn quad_vertex_pos_from_index(index: u32) -> vec2<f32>
{
    switch index {
        case 0u {
            return vec2<f32>(-0.003, 0.003);
        }
        case 1u {
            return vec2<f32>(-0.003, -0.003);
        }
        case 2u {
            return vec2<f32>(0.003, 0.003);
        }
        case 3u {
            return vec2<f32>(0.003, -0.003);
        }
        default { // should never happen
            return vec2<f32>(0.0, 0.0);
        }
    }
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
    out.clip_position += vec4<f32>(quad_vertex_pos_from_index(in_vertex_index), 0.0, 0.0);

    out.color = star.color;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    return in.color;
}