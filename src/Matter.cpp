#include "Matter.h"

#include "App.h"
#include "Material.h"

unsigned int Matter::gCurrentMatterID = 0;

unsigned int Matter::getNewID()
{
    unsigned int temp = gCurrentMatterID;
    gCurrentMatterID++;
    return temp;
}

Matter::Matter(App* app, const Material* material, bool floating) : mVertexShell(app), mRigidBody(app)
{
    mMaterial = material;
    mFloating = floating;
    mCollided = false;
    mMatterID = Matter::getNewID();
    mMaterial = new Material();
    m_debug_PressureVertexBufferID = 0;
}

Matter::~Matter()
{
    std::cout << "Matter being deleted..." << std::endl;
}

void Matter::setVoxelField(VoxelField voxelField)
{
    mVoxelField = voxelField;
}

void Matter::addVertex(GLfloat cX, GLfloat cY, GLfloat cZ, GLfloat nX, GLfloat nY, GLfloat nZ)
{
    mVertexShell.addVertex(cX, cY, cZ, nX, nY, nZ);
}

void Matter::setMaterial(const Material* material)
{
    mMaterial = material;
}

void Matter::addVertex(Vector3f coordinates, Vector3f normal)
{
    mVertexShell.addVertex(coordinates.x, coordinates.y, coordinates.z, normal.x, normal.y, normal.z);
}

VoxelField* Matter::getVoxelField()
{
    return &mVoxelField;
}

VertexShell* Matter::getVertexShell()
{
    return &mVertexShell;
}

const GLfloat* Matter::getColorPtr() const
{
    return mMaterial->getColor3();
}

bool Matter::nonTrivial(float energy)
{
    if(mVoxelField.getNumVoxels() > mMaterial->getTrivialMass())
    {
        float energyPerVoxel = (energy/mVoxelField.getNumVoxels())*32;
        return mMaterial->nonTrivial(energyPerVoxel);
    }
}

void Matter::import(const char* voxelFilename)
{
    mVoxelField.import(voxelFilename);
    mFilename = voxelFilename;
}

unsigned int Matter::getID()
{
    return mMatterID;
}

void Matter::setupHulls()
{
    //Create collision hulls
    GLsizei numHulls = mVoxelField.getNumHulls();

    for(int i = 0; i < numHulls; i++)
    {
        mCollisionHulls.push_back(new btConvexHullShape());
    }

    //DEBUG -- Create buffers to draw voxels from.
    m_debug_VoxelBufferIDs.resize(numHulls);
    m_debug_VoxelVertexArrays.resize(numHulls);
    glGenBuffers(numHulls, &m_debug_VoxelBufferIDs[0]);

    //DEBUG -- Create buffers to draw hulls from.
    m_debug_HullBufferIDs.resize(numHulls);
    m_debug_HullVertexArrays.resize(numHulls);
    glGenBuffers(numHulls, &m_debug_HullBufferIDs[0]);
}

const Material* Matter::getMaterial() const
{
    return mMaterial;
}

void Matter::addHullVertex(unsigned int hullIndex, Vector3f vertex)
{
    btVector3 vec(vertex.x, vertex.y, vertex.z);
    mCollisionHulls[hullIndex]->addPoint(vec);

    m_debug_HullVertexArrays[hullIndex].push_back(vertex.x);
    m_debug_HullVertexArrays[hullIndex].push_back(vertex.y);
    m_debug_HullVertexArrays[hullIndex].push_back(vertex.z);
}

void Matter::addVoxelVertex(unsigned int hullIndex, Vector3f vertex)
{
    /*
    btVector3 vec(vertex.x, vertex.y, vertex.z);
    mCollisionHulls[hullIndex]->addPoint(vec);
    */

    m_debug_VoxelVertexArrays[hullIndex].push_back(vertex.x);
    m_debug_VoxelVertexArrays[hullIndex].push_back(vertex.y);
    m_debug_VoxelVertexArrays[hullIndex].push_back(vertex.z);
}

void Matter::addPressureVertex(float pressure, float stress, float strength, Vector3f vertex)
{
    Vector3f cOfm = mVoxelField.getCenterOfMass();
    m_debug_PressureVertexArray.push_back(vertex.x - cOfm.x);
    m_debug_PressureVertexArray.push_back(vertex.y - cOfm.y);
    m_debug_PressureVertexArray.push_back(vertex.z - cOfm.z);

    float red = (pressure) / (mMaterial->getPressureLimit() * strength);
    //if(pressure >= mMaterial->getPressureLimit()) red = 1.0f;
    if(red > 1.0f) red = 1.0f;
    if(red < 0.0f) red = 0.0f;

    float blue = stress / (mMaterial->getStressLimit() * strength);
    if(blue > 1.0f) blue = 1.0f;
    if(blue < 0.0f) blue = 0.0f;

    float green = 1.0f - red - blue;
    if(green < 0.0f) green = 0.0f;

    m_debug_PressureVertexArray.push_back(red); //High pressure = Red.
    m_debug_PressureVertexArray.push_back(green); //Low pressure = Green.
    m_debug_PressureVertexArray.push_back(blue);
}

void Matter::addEnergyBridge(Vector3f vertex, bool local)
{
    Vector3f cOfm = mVoxelField.getCenterOfMass();
    m_debug_PressureVertexArray.push_back(vertex.x - cOfm.x
                                          );
    m_debug_PressureVertexArray.push_back(vertex.y - cOfm.y
                                          );
    m_debug_PressureVertexArray.push_back(vertex.z - cOfm.z
                                          );

    if(local)
    {
        m_debug_PressureVertexArray.push_back(1.0f); //Draw Bridge as White
        m_debug_PressureVertexArray.push_back(1.0f);
        m_debug_PressureVertexArray.push_back(1.0f);
    }
    else
    {
        m_debug_PressureVertexArray.push_back(0.0f); //Draw Bridge as Black
        m_debug_PressureVertexArray.push_back(0.0f);
        m_debug_PressureVertexArray.push_back(0.0f);
    }

}

void Matter::hullsDone()
{
    for(int i = 0; i < m_debug_VoxelBufferIDs.size(); i++)
    {
        glBindBuffer(GL_ARRAY_BUFFER, m_debug_VoxelBufferIDs[i]);
        glBufferData(GL_ARRAY_BUFFER, m_debug_VoxelVertexArrays[i].size()*sizeof(GLfloat), &m_debug_VoxelVertexArrays[i][0], GL_STATIC_DRAW);
    }
    for(int i = 0; i < m_debug_HullBufferIDs.size(); i++)
    {
        glBindBuffer(GL_ARRAY_BUFFER, m_debug_HullBufferIDs[i]);
        glBufferData(GL_ARRAY_BUFFER, m_debug_HullVertexArrays[i].size()*sizeof(GLfloat), &m_debug_HullVertexArrays[i][0], GL_STATIC_DRAW);
    }
}

void Matter::debugRenderVoxels()
{
    GLfloat colorArray[11][3];
    colorArray[0][0] = 1.0f; colorArray[0][1] = 0.0f; colorArray[0][2] = 0.0f;
    colorArray[1][0] = 0.0f; colorArray[1][1] = 1.0f; colorArray[1][2] = 0.0f;
    colorArray[2][0] = 0.0f; colorArray[2][1] = 0.0f; colorArray[2][2] = 1.0f;
    colorArray[3][0] = 1.0f; colorArray[3][1] = 1.0f; colorArray[3][2] = 0.0f;
    colorArray[4][0] = 0.0f; colorArray[4][1] = 1.0f; colorArray[4][2] = 1.0f;
    colorArray[5][0] = 1.0f; colorArray[5][1] = 0.0f; colorArray[5][2] = 1.0f;
    colorArray[6][0] = 1.0f; colorArray[6][1] = 1.0f; colorArray[6][2] = 1.0f;
    colorArray[7][0] = 0.0f; colorArray[7][1] = 0.0f; colorArray[7][2] = 0.0f;
    colorArray[8][0] = 0.7f; colorArray[8][1] = 0.7f; colorArray[8][2] = 0.3f;
    colorArray[9][0] = 0.3f; colorArray[9][1] = 0.3f; colorArray[9][2] = 0.7f;
    colorArray[10][0] = 0.7f; colorArray[10][1] = 0.3f; colorArray[10][2] = 0.7f;

    for(int i = 0; i < m_debug_VoxelBufferIDs.size(); i++)
    {
        //Mix color
        int colorIndex1 = i % 11;
        GLfloat color[3] = {colorArray[i][0], colorArray[i][1], colorArray[i][2]};
        glColor3fv(color);

        glBindBuffer(GL_ARRAY_BUFFER, m_debug_VoxelBufferIDs[i]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glDrawArrays(GL_POINTS, 0, m_debug_VoxelVertexArrays[i].size()/3);

        glDisableClientState(GL_VERTEX_ARRAY);
    }
}

void Matter::debugRenderHulls()
{
    GLfloat colorArray[11][3];
    colorArray[0][0] = 1.0f; colorArray[0][1] = 0.0f; colorArray[0][2] = 0.0f;
    colorArray[1][0] = 0.0f; colorArray[1][1] = 1.0f; colorArray[1][2] = 0.0f;
    colorArray[2][0] = 0.0f; colorArray[2][1] = 0.0f; colorArray[2][2] = 1.0f;
    colorArray[3][0] = 1.0f; colorArray[3][1] = 1.0f; colorArray[3][2] = 0.0f;
    colorArray[4][0] = 0.0f; colorArray[4][1] = 1.0f; colorArray[4][2] = 1.0f;
    colorArray[5][0] = 1.0f; colorArray[5][1] = 0.0f; colorArray[5][2] = 1.0f;
    colorArray[6][0] = 1.0f; colorArray[6][1] = 1.0f; colorArray[6][2] = 1.0f;
    colorArray[7][0] = 0.0f; colorArray[7][1] = 0.0f; colorArray[7][2] = 0.0f;
    colorArray[8][0] = 0.7f; colorArray[8][1] = 0.7f; colorArray[8][2] = 0.3f;
    colorArray[9][0] = 0.3f; colorArray[9][1] = 0.3f; colorArray[9][2] = 0.7f;
    colorArray[10][0] = 0.7f; colorArray[10][1] = 0.3f; colorArray[10][2] = 0.7f;

    for(int i = 0; i < m_debug_HullBufferIDs.size(); i++)
    {
        //Mix color
        int colorIndex1 = i % 11;
        GLfloat color[3] = {colorArray[i][0], colorArray[i][1], colorArray[i][2]};
        glColor3fv(color);

        glBindBuffer(GL_ARRAY_BUFFER, m_debug_HullBufferIDs[i]);

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glDrawArrays(GL_POINTS, 0, m_debug_HullVertexArrays[i].size()/3);

        glDisableClientState(GL_VERTEX_ARRAY);
    }
}

void Matter::clearPressureRendering()
{
    m_debug_PressureVertexArray.clear();
}

void Matter::setupPressureRendering()
{
    //If first time initialise Buffers
    if(m_debug_PressureVertexBufferID == 0)
    {
        glGenBuffers(1, &m_debug_PressureVertexBufferID);
    }
    glBindBuffer(GL_ARRAY_BUFFER, m_debug_PressureVertexBufferID);
    glBufferData(GL_ARRAY_BUFFER, m_debug_PressureVertexArray.size() * sizeof(GLfloat), &m_debug_PressureVertexArray[0], GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void Matter::debugRenderPressure()
{
    if(m_debug_PressureVertexBufferID != 0)
    {
        glBindBuffer(GL_ARRAY_BUFFER, m_debug_PressureVertexBufferID);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);

        glVertexPointer(3, GL_FLOAT, 6*sizeof(GLfloat), 0);
        glColorPointer(3, GL_FLOAT, 6*sizeof(GLfloat), (const GLvoid*) (sizeof(GLfloat)*3));

        glDrawArrays(GL_POINTS, 0, m_debug_PressureVertexArray.size()/6);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_COLOR_ARRAY);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

float Matter::getMass()
{
    return mVoxelField.getNumVoxels() * mMaterial->getDensity();
}

void Matter::setCollided(bool value)
{
    mCollided = value;
}

bool Matter::hasCollided()
{
    return mCollided;
}

btConvexHullShape* Matter::getHull(int index)
{
    return mCollisionHulls[index];
}

bool Matter::floats()
{
    return mFloating;
}

void Matter::setStartingPosition(Vector3 startingPos)
{
    mStartingWorldPosition = startingPos;
}

void Matter::beginProcessing()
{
    mVertexShell.startWrite();
}

void Matter::endProcessing(MatterNode* node)
{
    mVertexShell.endWrite();
    float mass = 0.0f;
    if(!floats())
    {
        mass = mVoxelField.getNumVoxels() * mMaterial->getDensity();
    }
    mRigidBody.setProperties(node, mCollisionHulls, mass, mStartingWorldPosition, mRigidBodyCenterOfMass);
}

RigidBody* Matter::getRigidBody()
{
    return &mRigidBody;
}

void Matter::processCollision(const btVector3& point, const btVector3& normal, const btScalar force)
{
        printf("Mass: %d Force: %f\n", getMass(), float(force));
}
