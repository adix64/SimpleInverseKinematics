#version 330

layout(location = 0) in vec3 v_position;
layout(location = 1) in vec3 v_normal;
layout(location = 2) in vec2 v_texture_coord;
layout(location = 3) in vec3 v_color;

// Uniform properties
uniform mat4 Model;
uniform mat4 View;
uniform mat4 Projection;
uniform int invertColor;

out vec2 texcoord;
out vec3 vcolor;
out vec3 world_normal;
void main()
{
	world_normal = vec3(Model * vec4(v_normal, 0.0));
	texcoord = v_texture_coord;
	
	vec3 col = v_color;
	if(invertColor != 0)		
		col = vec3(1,1,1) - col;
	
	
#define exponent 0.3
	vcolor = vec3(pow(col.r, exponent), pow(col.g, exponent), pow(col.b, exponent));

	gl_Position = Projection * View * Model * vec4(v_position, 1.0);
}
