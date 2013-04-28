#ifndef SCENENODE_H
#define SCENENODE_H

#include "VP.h"

#include <vector>
#include <boost/shared_ptr.hpp>

#include "Matrix4D.h"

class SceneNode;
class App;
typedef boost::shared_ptr<SceneNode> SceneNodePtr;

/*
Class: SceneNode
Description: SuperClass for all "Objects" in the game. Anything that requires per tick simulation and/or per frame drawing.
*/
class SceneNode
{
    public:
        SceneNode(App* app, GLenum renderPass);
        virtual ~SceneNode();

        virtual void onEvent();     //Called whenever an event occurs that this node was listening for.
        virtual void onInit();      //Should be called before this node is used as part of the simulation.
        virtual void onLoop(GLdouble deltaTime);      //Perform per tick simulation.
        virtual void simulateSelf(GLdouble deltaTime);
        virtual void onRender(GLenum renderPass);    //Render the object to the vertex buffer.
        virtual void renderSelf();
        virtual void onCleanup();   //Perform before destroying the actual object.

        void setTransform(const Matrix4D& transform);
        virtual Matrix4D getTransform();

        void addChild(SceneNodePtr child);

        GLboolean isDeleted();
        void deleteNode();

    protected:
        GLenum mRenderPass;                     //The render pass to draw this node in.

        Matrix4D mTransform;                    //The 4x4 Matrix representing the transform of this node.
        SceneNodePtr mParent;                   //Pointer to the parent of this node.
        std::vector<SceneNodePtr> mChildList;   //A list of the children of this node.

        GLboolean mDeleted;

        App* mApp;

    private:
};

#endif // SCENENODE_H
