layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;

layout (location = 0) out vec3 Normal;
layout (location = 1) out vec3 Color;

layout (std140, push_constant) uniform PushConstants {
	float size;
};

layout (std140, set = 0, binding = 0) uniform PassUBO {
	vec2 resolution;
	mat4 projection;
	mat4 view;
};

layout (std140, set = 0, binding = 1) uniform SimulationUBO {
	float time;
};

void main() {
    //float depth = model[0][0];
    gl_Position = projection * view * vec4(aPos.x,aPos.z,aPos.y, 1.0);
    gl_PointSize = 100 * size / gl_Position.w;  // / gl_Position.w; //pow(2, size)
    Color = vec3(aColor);
    Normal = aNormal;
}
