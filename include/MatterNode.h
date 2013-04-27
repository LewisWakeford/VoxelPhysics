#ifndef MATTERNODE_H
#define MATTERNODE_H

#include "SceneNode.h"

#include "Matter.h"
class MatterNode : public SceneNode
{
    public:
        MatterNode(App* app, GLenum renderPass, const Material* material, bool floats, const char* voxelFilename);
        MatterNode(App* app, GLenum renderPass, const Material* material, bool floats, VoxelField voxelField);
        virtual ~MatterNode();

        void setOffset(GLfloat x, GLfloat y, GLfloat z);
        void setOffset(Vector3 offset);

        Matter* getMatter();

    protected:
        Matter mMatter;
        virtual void renderSelf();
        virtual void simulateSelf(GLdouble deltaTime);

        Matrix4D getTransform();

        GLboolean mUpToDate;

        Vector3 mOffset;

    private:
};

#endif // MATTERNODE_H
