#include "SceneNode.h"
#include "Renderer.h"
#include "App.h"

SceneNode::SceneNode(App* app, GLenum renderPass)
{
    mApp = app;
    mTransform = Matrix4D::createIdentity();
    mRenderPass = renderPass;
    mDeleted = false;
}

SceneNode::~SceneNode()
{
    //dtor
}

void SceneNode::onEvent()
{

}

void SceneNode::onInit()
{

}

void SceneNode::onLoop(GLdouble deltaTime)
{
    if(!mDeleted)
    {
        //Sublclasses would perform some kind of simulation here.
        //The superclass doesn't bother.
        simulateSelf(deltaTime);

        //Perform simulation for all children.
        for(unsigned int i = 0; i < mChildList.size(); i++)
        {
            mChildList[i]->onLoop(deltaTime);
        }
    }
}

void SceneNode::onRender(GLenum renderPass)
{
    if(!mDeleted)
    {
        //Push Matrix onto renderer's stack.
    mApp->getRenderer()->pushMatrix(getTransform());

    if(mRenderPass == renderPass)
    {
        //Subclasses would actually render themselves here.
        //The super class doesn't bother.
        renderSelf();
    }



    //Render all Children.
    for(unsigned int i = 0; i < mChildList.size(); i++)
    {
        SceneNodePtr child = mChildList[i];
        if(child->isDeleted())
        {
            mChildList.erase(mChildList.begin()+i);
        }
        else
        {
            child->onRender(renderPass);
        }
    }

    //Pop Matrix off renderer's stack.
    mApp->getRenderer()->popMatrix();
    }
}

void SceneNode::onCleanup()
{

}

void SceneNode::setTransform(Matrix4D transform)
{
    mTransform = transform;
}

Matrix4D SceneNode::getTransform()
{
    return mTransform;
}

void SceneNode::renderSelf()
{
    //Do Nothing
}

void SceneNode::simulateSelf(GLdouble deltaTime)
{

}

void SceneNode::addChild(SceneNodePtr child)
{
    mChildList.push_back(child);
}

GLboolean SceneNode::isDeleted()
{
    return mDeleted;
}

void SceneNode::deleteNode()
{
    mDeleted = true;
}
