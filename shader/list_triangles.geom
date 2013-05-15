#version 330

layout(points) in;

in Voxel
{
	/*
	Each triangle is the edges it joins. There are 12 edges and so we need 12 bits. 4 For each edge.
	There are up to 32 voxels, which means we need 6 bits for each coord, which is 18.
	30 bits total.
	int format 00xxxxxxyyyyyyzzzzzz111122223333
	*/
	uint triangles[5];
	uint triangleCount;
	
} vVoxel[1];


layout(points, max_vertices = 5) out;

flat out uint gTriangle;

void main()
{

	for (int i = 0; i < 5;//vVoxel[0].triangleCount; 
	i++)
	{	
		gTriangle = vVoxel[0].triangles[i];
		//If all edges have number 12, then this triangle does not exist.
		if(!((gTriangle.x & 0xFFF)==0xCCC))
		{
			EmitVertex();
			EndPrimitive();
		}
		else
		{
			gTriangle = 0; //Make sure to overwrite every value in the output buffer.
			EmitVertex();
			EndPrimitive();
		}
	}

}
