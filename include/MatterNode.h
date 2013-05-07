#ifndef MATTERNODE_H
#define MATTERNODE_H

#include "SceneNode.h"

#include "Matter.h"
class MatterNode : public SceneNode
{
    public:
        MatterNode(App* app, GLenum renderPass, const MaterialPtr material, bool floats, const char* voxelFilename);
        MatterNode(App* app, GLenum renderPass, const MaterialPtr material, bool floats, VoxelField voxelField);
        virtual ~MatterNode();

        void setOffset(GLfloat x, GLfloat y, GLfloat z);
        void setOffset(Vector3 offset);
        void setLinearVelocity(btVector3 vel);
        void setMatrix(const Matrix4D& matrix);
        void setInitialForce(const Vector3f& force);

        btVector3 getLinearVelocity();

        Matter* getMatter();

        Matrix4D getTransform();
        void setTransform(const Matrix4D& transform);

    protected:
        Matter mMatter;
        virtual void renderSelf();
        virtual void simulateSelf(GLdouble deltaTime);

        Matrix4D mInitialMatrix;
        Vector3f mInitialForce;

        GLboolean mUpToDate;

        Vector3 mOffset;
        btVector3 mLinearVelocity;

    private:
};

typedef boost::shared_ptr<MatterNode> MatterNodePtr;

#endif // MATTERNODE_H
