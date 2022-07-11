struct StarSpec {
    orbit: vec4<f32>, // xy: xy radius of the ellipse z: elevation w: curve offset
};

struct Star {
    @location(0) position: vec3<f32>,
    @location(1) color: vec3<f32>,
};

struct SrcVertexBuffer {
    srcVertices: array<StarSpec>,
}

struct DstVertexBuffer {
    dstVertices: array<Star>,
}

@group(0) @binding(0)
var<storage> global: SrcVertexBuffer;

@group(0) @binding(1)
var<storage, read_write> global_1: DstVertexBuffer;

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

@compute @workgroup_size(64, 1, 1)
fn main(@builtin(global_invocation_id) param: vec3<u32>) {
    let id = param.x;

    let star = global.srcVertices[id];
    let orbit = star.orbit;

    // Theta is curve offset
    let theta = orbit.z;
    let sin_theta = sin(theta);
    let cos_theta = cos(theta);

    let phi = uniformFloat01(id) * 2.0 * 3.1415926;
    let sin_phi = sin(phi);
    let cos_phi = cos(phi);

    // Ellipse parameters
    let a = orbit.x;
    let b = orbit.y;

    let vx = a * cos_phi * cos_theta - b * sin_phi * sin_theta;
    let vy = a * cos_phi * sin_theta + b * sin_phi * cos_theta;

    let position = vec3<f32>(vx, vy, orbit.z);

    global_1.dstVertices[id].position = position;
    global_1.dstVertices[id].color = vec3<f32>(1.0, 1.0, 1.0);

    return;
}