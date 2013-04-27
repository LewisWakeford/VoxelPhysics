#include "MeshNode.h"
#include "App.h"
#include "Renderer.h"
#include "ResourceManager.h"

MeshNode::MeshNode(App* app, GLenum renderPass) : SceneNode(app, renderPass)
{
    //ctor
}

MeshNode::MeshNode(App* app, GLenum renderPass, GLint meshIndex) : SceneNode(app, renderPass)
{
    mMeshIndex = meshIndex;
}

MeshNode::~MeshNode()
{
    //dtor
}

void MeshNode::setMesh(GLint meshIndex)
{
    mMeshIndex = meshIndex;
}

GLint MeshNode::getMesh()
{
    return mMeshIndex;
}

void MeshNode::renderSelf()
{
    //mApp->getRenderer()->renderMesh(mApp->getResourceManager()->getMesh(mMeshIndex));
}
