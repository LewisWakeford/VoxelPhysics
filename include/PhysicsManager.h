#ifndef PHYSICSMANAGER_H
#define PHYSICSMANAGER_H

#include "btBulletDynamicsCommon.h"

#include "MatterCollisionSet.h"

class App;

void prePhysicsTickCallback(btDynamicsWorld* world, btScalar timeStep);
void postPhysicsTickCallback(btDynamicsWorld* world, btScalar timeStep);

class PhysicsManager
{
    public:
        PhysicsManager(App* app);
        virtual ~PhysicsManager();

        void simulate(double deltaTime);

        unsigned int addRigidBody(btRigidBody* rigidbody, btCollisionShape* shape);
        void removeRigidBody(btRigidBody* rigidbody, unsigned int shapeIndex);

        void preTickCallback(btDynamicsWorld *world, btScalar timeStep);
        void postTickCallback(btDynamicsWorld *world, btScalar timeStep);

    protected:

        App* mApp;

        btDefaultCollisionConfiguration* mCollisionConfiguration;

        btCollisionDispatcher* mDispatcher;

        btBroadphaseInterface* mBroadphaseInterface;

        btSequentialImpulseConstraintSolver* mSolver;

        btDiscreteDynamicsWorld* mDynamicsWorld;

        btAlignedObjectArray<btCollisionShape*> mCollisionShapesArray;

        std::vector<MatterCollisionSet> mCollisionSets;

        bool mPhysics;

    private:
};

#endif // PHYSICSMANAGER_H
