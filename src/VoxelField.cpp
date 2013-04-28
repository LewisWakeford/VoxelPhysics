#include "VoxelField.h"
#include "console.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

VoxelField::VoxelField() :
    mData(32, std::vector<std::vector<char>>(32, std::vector<char>(32, 0)))
{
    for(int z = 0; z < 32; z++)
    {
        for(int y = 0; y < 32; y++)
        {
            for(int x = 0; x < 32; x++)
            {
                mHulls[z][y][x] = -1;
            }
        }
    }

    mVoxels = 0;
    mNumHulls = 0;

}

VoxelField::~VoxelField()
{
    //dtor
}

void VoxelField::set(GLint x, GLint y, GLint z, GLubyte value)
{
    mData[z][y][x] = value;
}

void VoxelField::set(Vector3i voxelCoord, char value)
{
    mData[voxelCoord.z][voxelCoord.y][voxelCoord.x] = value;
}

void VoxelField::setHull(int z, int y, int x, int hullID)
{
    mHulls[z][y][x] = hullID;
}

void VoxelField::setNumberOfHulls(int numHulls)
{
    mNumHulls = numHulls;
}

GLubyte VoxelField::get(Vector3i coords) const
{
    return get(coords.x,coords.y,coords.z);
}

GLubyte VoxelField::getValue(Vector3i coords) const
{
    return getValue(coords.x,coords.y,coords.z);
}

GLubyte VoxelField::get(GLuint x, GLuint y, GLuint z) const
{
    if(x < 0)
    {
        x = 0;
    }
    else if(x > 31)
    {
        x = 31;
    }

    if(y < 0)
    {
        y = 0;
    }
    else if(y > 31)
    {
        y = 31;
    }

    if(z < 0)
    {
        z = 0;
    }
    else if(z > 31)
    {
        z = 31;
    }
    return mData[z][y][x];
}

GLubyte VoxelField::getValue(GLuint x, GLuint y, GLuint z) const
{
    if(x < 0)
    {
        return 0;
    }
    else if(x > 31)
    {
        return 0;
    }

    if(y < 0)
    {
        return 0;
    }
    else if(y > 31)
    {
        return 0;
    }

    if(z < 0)
    {
        return 0;
    }
    else if(z > 31)
    {
        return 0;
    }
    return mData[z][y][x];
}

int VoxelField::hullAt(GLuint x, GLuint y, GLuint z) const
{
    if(x < 0)
    {
        x = 0;
    }
    else if(x > 31)
    {
        x = 31;
    }
    if(y < 0)
    {
        y = 0;
    }
    else if(y > 31)
    {
        y = 31;
    }
    if(z < 0)
    {
        z = 0;
    }
    else if(z > 31)
    {
        z = 31;
    }
    return mHulls[z][y][x];
}

int VoxelField::hullAt(Vector3i coords) const
{
    return hullAt(coords.x,coords.y,coords.z);
}

GLfloat VoxelField::sample(GLfloat x, GLfloat y, GLfloat z)
{
    GLfloat xCell = x * 31;
    GLfloat yCell = y * 31;
    GLfloat zCell = z * 31;

    GLint xLow = floor(xCell);
    GLfloat xWeight = xCell - xLow;
    GLfloat xOpposite = 1.0f - xWeight;

    GLint yLow = floor(yCell);
    GLfloat yWeight = yCell - yLow;
    GLfloat yOpposite = 1.0f - yWeight;

    GLint zLow = floor(zCell);
    GLfloat zWeight = zCell - zLow;
    GLfloat zOpposite = 1.0f - zWeight;

    GLubyte sampleLLL = get(xLow, yLow, zLow);
    GLubyte sampleLLH = get(xLow, yLow, zLow+1);
    GLubyte sampleLHL = get(xLow, yLow+1, zLow);
    GLubyte sampleHLL = get(xLow+1, yLow, zLow);
    GLubyte sampleLHH = get(xLow, yLow+1, zLow+1);
    GLubyte sampleHHL = get(xLow+1, yLow+1, zLow);
    GLubyte sampleHLH = get(xLow+1, yLow, zLow+1);
    GLubyte sampleHHH = get(xLow+1, yLow+1, zLow+1);

    GLfloat result =    ((sampleLLL * xOpposite + sampleHLL * xWeight) * yOpposite +
                        (sampleLHL * xOpposite + sampleHHL * xWeight) * yWeight) * zOpposite +
                        ((sampleLLH * xOpposite + sampleHLH * xWeight) * yOpposite +
                        (sampleLHH * xOpposite + sampleHHH * xWeight) * yWeight) * zWeight;
    return result;
}

void VoxelField::printAs3DTexture(GLuint textureID)
{
    consolePrint("Binding texture");
    errorCheck(20);
    glBindTexture(GL_TEXTURE_3D, textureID);
    errorCheck(24);
    if(!(glIsTexture(textureID)==GL_TRUE)) consolePrint("Texture Binding Failed.");

    consolePrint("Creating texture array");
    GLfloat volumeData[32768*3];
    memset(volumeData, 0, sizeof(volumeData));

    for(int z = 0; z < 32; z++)
    {
        for(int y = 0; y < 32; y++)
        {
            for(int x = 0; x < 32; x++)
            {

                volumeData[(x*3)+(y*96)+(z*3072)] = (float)mData[z][y][x];
                volumeData[(x*3)+(y*96)+(z*3072)+1] = 0.0f;
                volumeData[(x*3)+(y*96)+(z*3072)+2] = 0.0f;

            }
        }
    }

    consolePrint("Copying array to texture");

    errorCheck(47);

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAX_LEVEL, 0);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_BASE_LEVEL, 0);
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB8, 32, 32, 32, 0, GL_RGB,
             GL_FLOAT, volumeData);




    errorCheck(50);

    consolePrint("Unbinding texture");
    glBindTexture(GL_TEXTURE_3D, 0);
    errorCheck(57);
}

void VoxelField::import(const char* filename)
{
    char currentByte;
    std::ifstream fileStream (filename);
    int x = 0;
    int y = 0;
    int z = 0;

    float sumX = 0;
    float sumY = 0;
    float sumZ = 0;
    int sum = 0;

    //Used for creating bounding box.
    unsigned int maxX = 0;
    unsigned int minX = 31;
    unsigned int maxY = 0;
    unsigned int minY = 31;
    unsigned int maxZ = 0;
    unsigned int minZ = 31;

    if(fileStream.is_open())
    {
        while(fileStream.good() && z < 32)
        {
            fileStream.get(currentByte);
            GLubyte value = 2;
            if(currentByte == '0') // 0
            {
                value = 0;
            }
            else if(currentByte == '1') // 1
            {
                value = 1;
                sumX += x;
                sumY += y;
                sumZ += z;

                //Update bounding box.
                if(x < minX)
                {
                    minX = x;
                }
                if(x > maxX)
                {
                    maxX = x;
                }
                if(y < minY)
                {
                    minY = y;
                }
                if(y > maxY)
                {
                    maxY = y;
                }
                if(z < minZ)
                {
                    minZ = z;
                }
                if(z > maxZ)
                {
                    maxZ = z;
                }

                sum++;
            }
            else if(currentByte == 'x') //X
            {
                x++;
                value = 2;
            }
            else if(currentByte == 'y') //Y
            {
                x = 0;
                y++;
                value = 2;
            }
            else if(currentByte == 'z') //Z
            {
                x = 0;
                y = 0;
                z++;
                value = 2;
            }

            if(value < 2)
            {
                mData[z][y][x] = value;
                x++;
            }

            if(x > 31)
            {
                x = 0;
                y++;
            }
            if(y > 31)
            {
                y = 0;
                z++;
            }
        }
        mCenterOfMass.x = sumX/sum;
        mCenterOfMass.y = sumY/sum;
        mCenterOfMass.z = sumZ/sum;
        mVoxels = sum;

        //Save bounding box values.
        mMinX = minX;
        mMaxX = maxX;
        mMinY = minY;
        mMaxY = maxY;
        mMinZ = minZ;
        mMaxZ = maxZ;
    }
    else
    {
        consolePrint("Faild to open .vox file.");
    }
    fileStream.close();
}


void VoxelField::buildStart()
{
    m_build_sumX = 0;
    m_build_sumY = 0;
    m_build_sumZ = 0;
    m_build_sum = 0;

    //Used for creating bounding box.
    m_build_maxX = 0;
    m_build_minX = 31;
    m_build_maxY = 0;
    m_build_minY = 31;
    m_build_maxZ = 0;
    m_build_minZ = 31;
}

void VoxelField::buildSetVoxel(Vector3i voxelCoord, char value)
{
    int& x = voxelCoord.x;
    int& y = voxelCoord.y;
    int& z = voxelCoord.z;
    if(value == 1)
    {
        m_build_sumX += x;
        m_build_sumY += y;
        m_build_sumZ += z;

        //Update bounding box.
        if(x < m_build_minX)
        {
            m_build_minX = x;
        }
        if(x > m_build_maxX)
        {
            m_build_maxX = x;
        }
        if(y < m_build_minY)
        {
            m_build_minY = y;
        }
        if(y > m_build_maxY)
        {
            m_build_maxY = y;
        }
        if(z < m_build_minZ)
        {
            m_build_minZ = z;
        }
        if(z > m_build_maxZ)
        {
            m_build_maxZ = z;
        }

        m_build_sum++;
    }
    set(voxelCoord, value);
}

void VoxelField::buildEnd()
{
    mCenterOfMass.x = m_build_sumX/m_build_sum;
    mCenterOfMass.y = m_build_sumY/m_build_sum;
    mCenterOfMass.z = m_build_sumZ/m_build_sum;
    mVoxels = m_build_sum;

    //Save bounding box values.
    mMinX = m_build_minX;
    mMaxX = m_build_maxX;
    mMinY = m_build_minY;
    mMaxY = m_build_maxY;
    mMinZ = m_build_minZ;
    mMaxZ = m_build_maxZ;
}


void VoxelField::createRandom()
{
    for(int z = 0; z < 32; z++)
    {
        for(int y = 0; y < 32; y++)
        {
            for(int x = 0; x < 32; x++)
            {
                mData[z][y][x] = rand() % 2;
            }
        }
    }
}

Vector3f VoxelField::getCenterOfMass()
{
    return mCenterOfMass;
}

unsigned int VoxelField::getNumVoxels()
{
    return mVoxels;
}

int VoxelField::getNumHulls()
{
    return mNumHulls;
}

bool VoxelField::isConvex()
{

}

BoundingBox VoxelField::createTransformedBoundingBox(const Matrix4D& objectToWorld, float margin)
{
    //Build an object space bounding box.

    Vector3f corners[8];
    corners[0] = Vector3f(mMinX-margin-mCenterOfMass.x, mMinY-margin-mCenterOfMass.y, mMinZ-margin-mCenterOfMass.z);
    corners[1] = Vector3f(mMaxX+margin-mCenterOfMass.x, mMinY-margin-mCenterOfMass.y, mMinZ-margin-mCenterOfMass.z);
    corners[2] = Vector3f(mMinX-margin-mCenterOfMass.x, mMaxY+margin-mCenterOfMass.y, mMinZ-margin-mCenterOfMass.z);
    corners[3] = Vector3f(mMinX-margin-mCenterOfMass.x, mMinY-margin-mCenterOfMass.y, mMaxZ+margin-mCenterOfMass.z);
    corners[4] = Vector3f(mMaxX+margin-mCenterOfMass.x, mMaxY+margin-mCenterOfMass.y, mMinZ-margin-mCenterOfMass.z);
    corners[5] = Vector3f(mMinX-margin-mCenterOfMass.x, mMaxY+margin-mCenterOfMass.y, mMaxZ+margin-mCenterOfMass.z);
    corners[6] = Vector3f(mMaxX+margin-mCenterOfMass.x, mMinY-margin-mCenterOfMass.y, mMaxZ+margin-mCenterOfMass.z);
    corners[7] = Vector3f(mMaxX+margin-mCenterOfMass.x, mMaxY+margin-mCenterOfMass.y, mMaxZ+margin-mCenterOfMass.z);

    //Transform into world space
    for(int i = 0; i < 8; i++)
    {
        corners[i] = objectToWorld.transformVertex(corners[i]);
    }

    //Recreate an AABB
    float minX = corners[0].x;
    float maxX = corners[0].x;
    float minY = corners[0].y;
    float maxY = corners[0].y;
    float minZ = corners[0].z;
    float maxZ = corners[0].z;

    for(int i = 1; i < 8; i++)
    {
        if(minX > corners[i].x) minX = corners[i].x;
        if(maxX < corners[i].x) maxX = corners[i].x;
        if(minY > corners[i].y) minY = corners[i].y;
        if(maxY < corners[i].y) maxY = corners[i].y;
        if(minZ > corners[i].z) minZ = corners[i].z;
        if(maxZ < corners[i].z) maxZ = corners[i].z;
    }

    return BoundingBox(minX, maxX, minY, maxY, minZ, maxZ);
}


Vector3i VoxelField::getVoxelClosestTo(const Matrix4D& worldToObject, Vector3f point)
{
    Vector3f pointInObject = worldToObject.transformVertex(point);

    //Round to nearest
    Vector3i closest(floor(pointInObject.x+0.5), floor(pointInObject.y+0.5), floor(pointInObject.z+0.5));

    return closest;
}

void VoxelField::getPotentialBridges(const Matrix4D& worldToObject, Vector3f directionOfTravel, std::vector<Vector3i>& potentialBridges)
{
    Vector3f localDirection = worldToObject.rotateVector(directionOfTravel);

    int xDir = 0;
    int startX = 0;

    if(localDirection.x > 0)
    {
        xDir = -1;
        startX = mMaxX;
    }
    else if(localDirection.x < 0)
    {
        xDir = 1;
        startX = mMinX;
    }


    if(xDir != 0)
    {
        for(unsigned int z = 0; z < 32; z++)
        {
            for(unsigned int y = 0; y < 32; y++)
            {
                bool found = false;
                for(unsigned int x = startX; x >= 0 && x < 32 && !found; x += xDir)
                {
                    if(get(x, y, z) == 1)
                    {
                        potentialBridges.push_back(Vector3i(x, y, z));
                        found = true;
                    }
                }
            }
        }
    }

    int yDir = 0;
    int startY = 0;

    if(localDirection.y > 0)
    {
        yDir = -1;
        startY = mMaxY;
    }
    else if(localDirection.y < 0)
    {
        yDir = 1;
        startY = mMinY;
    }


    if(yDir != 0)
    {
        for(unsigned int z = 0; z < 32; z++)
        {
            for(unsigned int x = 0; x < 32; x++)
            {
                bool found = false;
                for(unsigned int y = startY; y >= 0 && y < 32 && !found; y += yDir)
                {
                    if(get(x, y, z) == 1)
                    {
                        potentialBridges.push_back(Vector3i(x, y, z));
                        found = true;
                    }
                }
            }
        }
    }

    int zDir = 0;
    int startZ = 0;

    if(localDirection.z > 0)
    {
        zDir = -1;
        startZ = mMaxZ;
    }
    else if(localDirection.z < 0)
    {
        zDir = 1;
        startZ = mMinZ;
    }


    if(zDir != 0)
    {
        for(unsigned int y = 0; y < 32; y++)
        {
            for(unsigned int x = 0; x < 32; x++)
            {
                bool found = false;
                for(unsigned int z = startZ; z >= 0 && z < 32 && !found; z += zDir)
                {
                    if(get(x, y, z) == 1)
                    {
                        potentialBridges.push_back(Vector3i(x, y, z));
                        found = true;
                    }
                }
            }
        }
    }

}

Vector3i VoxelField::getFirstFree()
{
    for(int z = 31; z >= 0; z--)
    {
        for(int y = 0; y < 32; y++)
        {
            for(int x = 0; x < 32; x++)
            {
                if(mData[z][y][x] == (GLubyte)1 && mHulls[z][y][x] == -1)
                {
                    return Vector3i(x, y, z);
                }
            }
        }
    }
    return Vector3i(-1, -1, -1);
}

BoundingBox::BoundingBox(float minX, float maxX,
                    float minY, float maxY,
                    float minZ, float maxZ)
{
    mMinX = minX;
    mMaxX = maxX;
    mMinY = minY;
    mMaxY = maxY;
    mMinZ = minZ;
    mMaxZ = maxZ;
}

BoundingBox::~BoundingBox()
{

}

bool BoundingBox::pointInside(Vector3f point)
{
    bool withinX = false;
    bool withinY = false;
    bool withinZ = false;

    if(point.x >= mMinX && point.x <= mMaxX)
    {
        withinX = true;
    }
    if(point.y >= mMinY && point.y <= mMaxY)
    {
        withinY = true;
    }
    if(point.z >= mMinZ && point.z <= mMaxZ)
    {
        withinZ = true;
    }

    if(withinX && withinY && withinZ)
    {
        return true;
    }
    else
    {
        return false;
    }
}
