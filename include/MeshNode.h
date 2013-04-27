#ifndef MESHNODE_H
#define MESHNODE_H

#include "SceneNode.h"

class App;

class MeshNode : public SceneNode
{
    public:
        MeshNode(App* app, GLenum renderPass);
        MeshNode(App* app, GLenum renderPass, GLint meshIndex);
        virtual ~MeshNode();

        void setMesh(GLint meshIndex);
        GLint getMesh();

       virtual void renderSelf();

    protected:
        GLint mMeshIndex;

    private:
};

#endif // MESHNODE_H
