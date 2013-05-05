#version 130

layout(points) in;

in triangleVerts
{
	vec3 coord1;
	vec3 normal1;
	vec3 coord2;
	vec3 normal2;
	vec3 coord3;
	vec3 normal3;
} vertsIn[1];

layout(points, max_vertices = 3) out;

out vec3 position;
out vec3 normal;

void main()
{
	position = vertsIn[0].coord1;
	normal = vertsIn[0].normal1;
	EmitVertex();
	
	position = vertsIn[0].coord2;
	normal = vertsIn[0].normal2;
	EmitVertex();
	
	position = vertsIn[0].coord3;
	normal = vertsIn[0].normal3;
	EmitVertex();
	
	EndPrimitive();
}