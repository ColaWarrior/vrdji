#version 300 es
#extension GL_OVR_multiview : enable
#extension GL_OVR_multiview2 : enable
#extension GL_OVR_multiview_multisampled_render_to_texture : enable

layout(num_views = 2) in;

uniform mat4 matrix[2];
layout(location = 0) in vec3 v3Position;

void main() {
    gl_Position = matrix[gl_ViewID_OVR] * vec4(v3Position.xyz, 1);
}