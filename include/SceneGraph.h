#ifndef SCENEGRAPH_H
#define SCENEGRAPH_H

#include "VP.h"

#include <boost/shared_ptr.hpp>

//Forward Declartions
class SceneNode;
typedef boost::shared_ptr<SceneNode> SceneNodePtr;

//Class: SceneGraph
//Stores a reference to the root Scene Node and can therefore perform operations on the entire graph.
class SceneGraph
{
    public:
        SceneGraph();
        virtual ~SceneGraph();

        void onInit();      //Initialise all Nodes in the graph.
        void onLoop(GLdouble deltaTime);      //Perform per tick simulation on all Nodes in the graph.
        void onRender(GLenum renderPass);    //Render all nodes.
        void onCleanup();   //Perform pre-deletion actions on all nodes.

        SceneNodePtr getRoot();
        void setRoot(SceneNodePtr root);

    protected:
        SceneNodePtr mRoot; //Pointer to the root node.
    private:
};

#endif // SCENEGRAPH_H
