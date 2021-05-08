#version 300 es
uniform mat4 matrix;
layout(location = 0) in vec3 v3Position;

void main() {
    gl_Position = matrix * vec4(v3Position.xyz, 1.0f);
}