#version 300 es
uniform mat4 matrix;
layout(location = 0) in vec3 v3Position;
layout(location = 2) in vec2 v2Coord;
out vec2 v2fCoord;
void main() {
    v2fCoord = vec2(v2Coord.s, v2Coord.t);
    gl_Position = matrix * vec4(v3Position.xyz, 1.0f);
}