#version 120

varying vec3 normal;

uniform vec3 worldLightVector;
uniform vec4 worldLightColor;
uniform vec4 ambientColor;

void main()
{
	vec3 normal = normalize(normal);
	
	float diffuseTerm = clamp(dot(normal, normalize(worldLightVector)), 0.0, 1.0);
	
	gl_FragColor = (gl_Color * ambientColor) + (gl_Color * worldLightColor * diffuseTerm);
}