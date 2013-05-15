#ifndef VOXELCONVERTER_H
#define VOXELCONVERTER_H

#include <vector>
#include <initializer_list>

#include "VP.h"
#include "ShaderProgram.h"
#include "Buffer.h"
#include "Vector3.h"
#include "App.h"

class VoxelField;
class VertexShell;
class Matter;
class MatterNode;

class ShaderProgram;

const unsigned int MAX_CELLS = 29791;

class RegularCellData
{
    public:

        RegularCellData(GLuint geometryCounts, std::initializer_list<unsigned int> init);
        virtual ~RegularCellData();

        void init(GLuint geometryCounts, const GLuint* vertexIndex);

        GLuint	mGeometryCounts;		/* High nibble is vertex count, low nibble is triangle count. */
        GLuint	mVertexIndex[15];	/* Groups of 3 indexes giving the triangulation. */

        GLuint getVertexCount();

        GLuint getTriangleCount();
};

struct ListTrianglesOutput
{
    public:
        GLuint triangleCount;
        GLuint triangles[5];
};

class VoxelConverter
{
    public:
        VoxelConverter(App* app);
        VoxelConverter(App* app, GLfloat spacing);
        virtual ~VoxelConverter();

        void convert(MatterNode* matter);

        void drawLast();

        GLboolean initGPU(const char* lst_tri_vert, const char* lst_tri_geom,
                        const char* gen_vert_vert, const char* gen_vert_geom);

        void initCPU();

    protected:

        GLuint mTransformFeedback[];

        //The buffer containing blank vertices, is the same for every pass.
        GLuint mInitialDataBuffer;

        //The buffer containing the triangle marker uints, output of list_triangles.
        GLuint mTriangleBuffer;
        std::vector<GLuint> mTriangleArray;

        //The buffer containing the finished vertexs.
        std::vector<GLfloat> mVertexArray;

        //Volume texture, overwritten by each new voxel field.
        GLuint mVolumeTexture;

        //When set to true runs the algorithm on the CPU instead of the GPU.
        GLboolean mUseCPU;

        void convertGPU(MatterNode* matter);
        void listTrianglesGPU(unsigned int numCells);
        void genVerticesGPU(unsigned int numCells, Buffer* outputBuffer);

        void convertCPU(MatterNode* matter);
        void listTrianglesCPU(Matter* matter);
        void genVerticesCPU(Matter* matter);

        //Only do the processing huls part of the generation.
        //For when using GPU acceleration.
        void processHulls(Matter* matter);

        void addVoxelVertex(Matter* matter, Vector3i voxelCoord);
        void addHullVertex(Matter* matter, const Vector3i& voxelCoord, const Vector3f& normal);

        ShaderProgram mListTriangles;
        ShaderProgram mGenVertices;

        GLfloat mVoxelSpacing;
        GLfloat mHalfVoxel;


        GLfloat mXOffset;
        GLfloat mYOffset;
        GLfloat mZOffset;
        GLfloat mOffset[3];

        App* mApp;

    private:

    /*
    IMPORTANT NOTE:
        All reference tables as well as the helper data structures adapted from code provided by Eric Lengyel.
    */

    static GLuint getEdgeNumber(GLuint cornerIndices);

	/*
	The regularCellClass table maps an 8-bit regular Marching Cubes case index to
	an equivalencecellClassindex. Even though there are 18 equivalence classes in our
	modified Marching Cubes algorithm, a couple of them use the same exact triangulations,
	just wi th different vertex locations. We combined those classes for this table so
	that thecellClassindex ranges from 0 to 15.
	*/

    static const GLuint regularCellClass[256];

	/*
	The regularCellData table holds the triangulation data for all 16 distinct classes to
	which a case can be mapped by the regularCellClass table.
	*/

    static RegularCellData regularCellData[16];

    /*
        The regularVertexData table gives the vertex locations for every one of the 256 possible
        cases in the modified Marching Cubes algorithm. Each 16-bit value also provides information
        about whether a vertex can be reused from a neighboring cell. See Section 3.3 for details.
        The low byte contains the indexes for the two endpoints of the edge on which the vertex lies,
        as numbered in Figure 3.7. The high byte contains the vertex reuse data shown in Figure 3.8.

        We don't need the mapping code currently
    */

    static const GLuint regularVertexData[3072];

    //Reference table storing all the possible "cube normals" used in convex decomp.
    Vector3f mCubeNormals[256];
};

#endif // VOXELCONVERTER_H
