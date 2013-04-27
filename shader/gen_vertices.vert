#version 130

/*00xxxxxxyyyyyyzzzzzz111122223333*/
layout (location = 0) in uint triangle;
uniform usampler3D densityVol;
uniform float voxelSpace;

out triangleVerts
{
	vec3 coord1;
	vec3 normal1;
	vec3 coord2;
	vec3 normal2;
	vec3 coord3;
	vec3 normal3;
} vertsOut;

void main()
{
	uint voxX = (triangle >> 25) & 31;
	uint voxY = (triangle >> 20) & 31;
	uint voxZ = (triangle >> 15) & 31;
	
	uint edge[3];
	edge[0] = (triangle >> 8) & 0xF;
	edge[1] = (triangle >> 4) & 0xF;
	edge[2] = triangle & 0xF;
	
	/*Place a vertex on each edge.*/
	vec3 coord[3];
	coord[0] = vec3(float(voxX)*voxelSpace,float(voxY)*voxelSpace,float(voxZ)*voxelSpace);
	coord[1] = vec3(float(voxX)*voxelSpace,float(voxY)*voxelSpace,float(voxZ)*voxelSpace);
	coord[2] = vec3(float(voxX)*voxelSpace,float(voxY)*voxelSpace,float(voxZ)*voxelSpace);
	
	vec3 normal[3];

	float halfSpace = voxelSpace*float(0.5);
	float totalSpace = voxelSpace*32.0f;
	float spaceRatio = 1.0f/32.0f;
	
	for(int i = 0; i < 3; i++)
	{
		if(edge[i] == 0)
		{
			coord[i].x = coord[i].x + halfSpace;
		}
		else if(edge [i] == 1)
		{
			coord[i].y = coord[i].y + halfSpace;
		}
		else if(edge [i] == 2)
		{
			coord[i].x = coord[i].x + halfSpace;
			coord[i].y = coord[i].y + voxelSpace;
		}
		else if(edge [i] == 3)
		{
			coord[i].x = coord[i].x + voxelSpace;
			coord[i].y = coord[i].y + halfSpace;
		}
		else if(edge [i] == 4)
		{
			coord[i].x = coord[i].x + halfSpace;
			coord[i].z = coord[i].z + voxelSpace;
		}
		else if(edge [i] == 5)
		{
			coord[i].y = coord[i].y + halfSpace;
			coord[i].z = coord[i].z + voxelSpace;
		}
		else if(edge [i] == 6)
		{
			coord[i].x = coord[i].x + halfSpace;
			coord[i].y = coord[i].y + voxelSpace;
			coord[i].z = coord[i].z + voxelSpace;
		}
		else if(edge [i] == 7)
		{
			coord[i].x = coord[i].x + voxelSpace;
			coord[i].y = coord[i].y + halfSpace;
			coord[i].z = coord[i].z + voxelSpace;
		}
		else if(edge [i] == 8)
		{
			coord[i].z = coord[i].z + halfSpace;
		}
		else if(edge [i] == 9)
		{
			coord[i].y = coord[i].y + voxelSpace;
			coord[i].z = coord[i].z + halfSpace;
		}
		else if(edge [i] == 10)
		{
			coord[i].x = coord[i].x + voxelSpace;
			coord[i].y = coord[i].y + voxelSpace;
			coord[i].z = coord[i].z + halfSpace;
		}
		else if(edge [i] == 11)
		{
			coord[i].x = coord[i].x + voxelSpace;
			coord[i].z = coord[i].z + halfSpace;
		}
		
		if(edge[i] != 12)
		{
			/*Create a normal from the rate of change in x, y and z*/
			float xSample = ((coord[i].x)/totalSpace)+halfSpace;
			float ySample = ((coord[i].y)/totalSpace)+halfSpace;
			float zSample = ((coord[i].z)/totalSpace)+halfSpace;
			
			vec4 base = texture3D(densityVol, vec3(xSample, ySample, zSample));
			normal[i].x = texture3D(densityVol, vec3(xSample+spaceRatio, ySample, zSample)).r - base.r;
			normal[i].y = texture3D(densityVol, vec3(xSample, ySample+spaceRatio, zSample)).r - base.r;
			normal[i].z = texture3D(densityVol, vec3(xSample, ySample, zSample+spaceRatio)).r - base.r;
			normal[i] = normalize(normal[i]);
		}
		
	}
	
	
	
	/*Output*/
	vertsOut.coord1 = coord[0];
	vertsOut.coord2 = coord[1];
	vertsOut.coord3 = coord[2];
	
	vertsOut.normal1 = normal[0];
	vertsOut.normal2 = normal[1];
	vertsOut.normal3 = normal[2];
}