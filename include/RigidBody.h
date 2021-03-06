#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include "Matrix4D.h"
#include "btBulletDynamicsCommon.h"

#include <vector>


class App;
class MatterNode;

/**
    Class: RigidBody
    Wraps a bullet physics rigid body.
*/
class RigidBody
{
    public:
        RigidBody(App* app);
        virtual ~RigidBody();

        //Create a new collision shape for this RB.
        void setProperties(MatterNode* matter, const std::vector<btConvexHullShape*>& hulls, float mass, Vector3f startingWorldPos, Vector3f centerOfMass);

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

        //Starting transform and velocity.
        btTransform* mPreparedTransform;
        btVector3 mPreparedVelocity;

        bool mTransformOutdated;
        bool mInverseTransformOutdated;

        unsigned int mShapeIndex;

    private:
};

#endif // RIGIDBODY_H
