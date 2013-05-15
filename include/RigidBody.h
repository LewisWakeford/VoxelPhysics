#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Matrix4D.h"
#include "btBulletDynamicsCommon.h"

#include <vector>


class App;
class MatterNode;

class RigidBody
{
    public:
        RigidBody(App* app);
        virtual ~RigidBody();

        //Create a new collision shape for this RB.
        void setProperties(MatterNode* matter, const std::vector<btConvexHullShape*>& hulls, float mass, Vector3 startingWorldPos, Vector3 centerOfMass);

        void setTransform(Matrix4D transform);
        const Matrix4D& getTransform();
        const Matrix4D& getInvertedTransform();
        void applyForce(const Vector3f& force);
        void setVelocity(const Vector3f& velocity);

    protected:
        App* mApp;
        btRigidBody* mRigidBody;

        Matrix4D mTransform;
        Matrix4D mInvertedTransform;
        btTransform* mPreparedTransform;
        btVector3 mPreparedVelocity;

        bool mTransformOutdated;
        bool mInverseTransformOutdated;

        unsigned int mShapeIndex;

    private:
};

#endif // RIGIDBODY_H
