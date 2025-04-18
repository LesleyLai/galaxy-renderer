struct StarSpec {
    orbit: vec4<f32>, // xy: xy radius of the ellipse z: elevation w: curve offset
    color: vec4<f32>,
};

struct Star {
    @location(0) position: vec4<f32>,
    @location(1) color: vec4<f32>,
};

struct IndirectBuffer {
    vertex_count: u32,
    instance_count: atomic<u32>,
    first_vertex: u32,
    first_instance: u32,
}

struct SrcVertexBuffer {
    srcVertices: array<StarSpec>,
}

struct DstVertexBuffer {
    dstVertices: array<Star>,
}

@group(0) @binding(0)
var<storage, read_write> indirect_buffer: IndirectBuffer;

@group(0) @binding(1)
var<storage> global: SrcVertexBuffer;

@group(0) @binding(2)
var<storage, read_write> global_1: DstVertexBuffer;

@group(0) @binding(3)
var<uniform> time: f32;

const TWO_PI: f32 = 6.28318531;

fn pcg(seed: u32) -> u32
{
    let state = seed * 747796405u + 2891336453u;
    let word = ((state >> ((state >> 28u) + 4u)) ^ state) * 277803737u;
    return (word >> 22u) ^ word;
}

fn uniformFloat01(state: u32) -> f32
{
    let state2 = pcg(state);
    return f32(state2) / 4294967296.0;
}

// Normalizes an angle to the rangle of [0, 2 pi)
fn wrap_zero_two_pi(theta: f32) -> f32
{
    var result = theta;
    while (result < 0.0) { result += TWO_PI; }
    while (result >= TWO_PI) { result -= TWO_PI; }
    return result;
}


@compute @workgroup_size(64, 1, 1)
fn main(@builtin(global_invocation_id) param: vec3<u32>, @builtin(num_workgroups) workgroups_count: vec3<u32>) {
    let id = param.x;

    if (id >= workgroups_count.x) {
      return;
    }

    let star = global.srcVertices[id];
    let orbit = star.orbit;

    // Theta is curve offset
    let theta = orbit.w;
    let sin_theta = sin(theta);
    let cos_theta = cos(theta);

    let phi = wrap_zero_two_pi(uniformFloat01(id) * TWO_PI - time * 0.3 * min(0.1, orbit.x));

    let sin_phi = sin(phi);
    let cos_phi = cos(phi);

    // Ellipse parameters
    let a = orbit.x;
    let b = orbit.y;

    let vx = a * cos_phi * cos_theta - b * sin_phi * sin_theta;
    let vy = a * cos_phi * sin_theta + b * sin_phi * cos_theta;

    let position = vec4<f32>(vx, vy, orbit.z, f32(id));

    atomicAdd(&indirect_buffer.instance_count, 1u);

    global_1.dstVertices[id].position = position;
    global_1.dstVertices[id].color = star.color;

    return;
}