#include "SceneGraph.h"
#include "SceneNode.h"

SceneGraph::SceneGraph()
{
    //ctor
}

SceneGraph::~SceneGraph()
{
    //dtor
}

void SceneGraph::onLoop(GLdouble deltaTime)
{
    mRoot->onLoop(deltaTime);
}

void SceneGraph::onRender(GLenum renderPass)
{
    mRoot->onRender(renderPass);
}

SceneNodePtr SceneGraph::getRoot()
{
    return mRoot;
}

void SceneGraph::setRoot(SceneNodePtr root)
{
    mRoot = root;
}
