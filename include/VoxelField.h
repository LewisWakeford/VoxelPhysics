#ifndef VOXELFIELD_H
#define VOXELFIELD_H

#include "VP.h"
#include "Vector3.h"
#include "Matrix4D.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

class BoundingBox
{
    public:
        BoundingBox(float minX, float maxX,
                    float minY, float maxY,
                    float minZ, float maxZ);
        virtual ~BoundingBox();
        bool pointInside(Vector3f point);

    protected:
        float mMinX;
        float mMaxX;
        float mMinY;
        float mMaxY;
        float mMinZ;
        float mMaxZ;
};

class VoxelField
{
    public:
        VoxelField();
        virtual ~VoxelField();

        void printAs3DTexture(GLuint textureID);

        void set(GLint z, GLint y, GLint x, GLubyte value);
        void set(Vector3i voxelCoord, char value);
        void setHull(int z, int y, int x, int hullID);
        void setNumberOfHulls(int numHulls);

        GLfloat sample(GLfloat x, GLfloat y, GLfloat z);
        GLubyte get(Vector3i coordinates) const;
        GLubyte getValue(Vector3i coordinates) const;
        GLubyte get(GLuint x, GLuint y, GLuint z) const;
        GLubyte getValue(GLuint x, GLuint y, GLuint z) const;

        unsigned int getNumVoxels();

        int hullAt(GLuint x, GLuint y, GLuint z) const;
        int hullAt(Vector3i coordinates) const;
        int getNumHulls();
        Vector3i getFirstFree();

        //Return the coordinate in voxel space of the voxel closest to the point.
        Vector3i getVoxelClosestTo(const Matrix4D& worldToObject, Vector3f point);

        //List all the voxels that might be a bridge, given that the field is moving in the direction given.
        void getPotentialBridges(const Matrix4D& worldToObject, Vector3f directionOfTravel, std::vector<Vector3i>& potentialBridges);

        void import(const char* filename);

        void buildStart();
        void buildSetVoxel(Vector3i voxelCoord, char value);
        void buildEnd();

        void createRandom();

        int getTotalMass();
        Vector3f getCenterOfMass();
        bool isConvex();

        //Build collision hulls using the voxel data. CD on the finished vertices is way too slow.
        void generateHulls();

        //Create a AABB in world coords with the margin of margin.
        BoundingBox createTransformedBoundingBox(const Matrix4D& objectToWorld, float margin);

    protected:
        std::vector<std::vector<std::vector<char>>> mData;
        int mHulls[32][32][32]; //Sub-shape mappings
        int mNumHulls;

        Vector3f mCenterOfMass;
        unsigned int mVoxels; //Total number of "full" voxels.

        //Bounding box
        unsigned int mMinX;
        unsigned int mMaxX;
        unsigned int mMinY;
        unsigned int mMaxY;
        unsigned int mMinZ;
        unsigned int mMaxZ;

        float m_build_sumX;
    float m_build_sumY;
    float m_build_sumZ;
    int m_build_sum;

    //Used for creating bounding box.
    unsigned int m_build_maxX;
    unsigned int m_build_minX;
    unsigned int m_build_maxY;
    unsigned int m_build_minY;
    unsigned int m_build_maxZ;
    unsigned int m_build_minZ;

    private:
};



#endif // VOXELFIELD_H
