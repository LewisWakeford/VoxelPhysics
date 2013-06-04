#ifndef MATTERNODE_H
#define MATTERNODE_H

#include "SceneNode.h"

#include "Matter.h"

/**
    Class: MatterNode
    A scenenode that manages a matter object.
    TODO: Cleanup Matter and MatterNode classes, as the destinction betweeen them is blurry.
*/
class MatterNode : public SceneNode
{
    public:
        MatterNode(App* app, GLenum renderPass, const MaterialPtr material, bool floats, const char* voxelFilename);
        MatterNode(App* app, GLenum renderPass, const MaterialPtr material, bool floats, VoxelField voxelField);
        virtual ~MatterNode();

        void setOffset(GLfloat x, GLfloat y, GLfloat z);
        void setOffset(Vector3f offset);
        void setLinearVelocity(btVector3 vel);
        void setMatrix(const Matrix4D& matrix);
        void setInitialForce(const Vector3f& force); //Set the initial force the Matter Object will recieve when simulation begins.
        void setEnergy(const Vector3f& energyVector); //Convert energy back into velocity.

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

        Vector3f mOffset;
        btVector3 mLinearVelocity;

    private:
};

typedef boost::shared_ptr<MatterNode> MatterNodePtr;

#endif // MATTERNODE_H
