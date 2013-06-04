#include "MatterNode.h"

#include "App.h"
#include "Renderer.h"
#include "VoxelConverter.h"

MatterNode::MatterNode(App* app, GLenum renderPass, const MaterialPtr material, bool floats, const char* voxelFilename): SceneNode(app, renderPass), mMatter(app, material, floats)
{
    mMatter.import(voxelFilename);
    mMatter.setMaterial(material);
    mUpToDate = false;
    mInitialMatrix = Matrix4D::createIdentity();
}

MatterNode::MatterNode(App* app, GLenum renderPass, const MaterialPtr material, bool floats, VoxelField voxelField): SceneNode(app, renderPass), mMatter(app, material, floats)
{
    mMatter.setVoxelField(voxelField);
    mMatter.setMaterial(material);
    mUpToDate = false;
}

MatterNode::~MatterNode()
{
    std::cout << "Matter Node being deleted..." << std::endl;
}

void MatterNode::renderSelf()
{
    mApp->getRenderer()->renderMatter(mMatter);
}

void MatterNode::simulateSelf(GLdouble deltaTime)
{
    if(!mUpToDate)
    {
        mApp->getVoxelConverter()->convert(this);
        //setTransform(mInitialMatrix);
        mMatter.getRigidBody()->applyForce(mInitialForce);
        mUpToDate = true;
    }
}

Matrix4D MatterNode::getTransform()
{
    return mMatter.getRigidBody()->getTransform();
}

void MatterNode::setTransform(const Matrix4D& transform)
{
    mMatter.getRigidBody()->setTransform(transform);
}

Matter* MatterNode::getMatter()
{
    return &mMatter;
}

void MatterNode::setOffset(GLfloat x, GLfloat y, GLfloat z)
{
    Vector3f offset = {x, y, z};
    setOffset(offset);
}

void MatterNode::setOffset(Vector3f offset)
{
    mOffset = offset;
    mMatter.setStartingPosition(offset);
}

void MatterNode::setLinearVelocity(btVector3 vel)
{
    mLinearVelocity = vel;
}

btVector3 MatterNode::getLinearVelocity()
{
    return mLinearVelocity;
}

void MatterNode::setMatrix(const Matrix4D& matrix)
{
    mInitialMatrix = matrix;
}

void MatterNode::setInitialForce(const Vector3f& force)
{
    mInitialForce = force;
}

void MatterNode::setEnergy(const Vector3f& energyVector)
{
    float speed = sqrtf((2 * energyVector.length())/mMatter.getMass());
    setInitialForce(energyVector.normalized() * speed * mMatter.getMass());
}
