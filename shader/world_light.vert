#version 120

varying vec3 normal;
varying vec3 lightingVector;

uniform vec4 worldLightVector;
uniform vec4 worldLightColor;
uniform vec4 ambientColor;

void main()
{
	gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
	
	normal = gl_Normal;
	
	gl_FrontColor = gl_Color;
}