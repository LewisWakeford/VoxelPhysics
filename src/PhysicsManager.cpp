#include "PhysicsManager.h"

#include "console.h"
#include "Matter.h"
#include "MatterNode.h"
#include "App.h"
#include "DestructionEngine.h"

void physicsTickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    PhysicsManager* man = static_cast<PhysicsManager*>(world->getWorldUserInfo());
    man->tickCallback(world, timeStep);
}

PhysicsManager::PhysicsManager(App* app)
{
    mPhysics = true;

    mApp = app;

    mCollisionConfiguration = new btDefaultCollisionConfiguration(); //Attempt to roll own later
    mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);
    mBroadphaseInterface = new btDbvtBroadphase();
    mSolver = new btSequentialImpulseConstraintSolver;
    mDynamicsWorld = new btDiscreteDynamicsWorld(mDispatcher, mBroadphaseInterface, mSolver, mCollisionConfiguration);

    //Set callback.
    mDynamicsWorld->setInternalTickCallback(physicsTickCallback, static_cast<void *>(this));
    mDynamicsWorld->setGravity(btVector3(0.0f, 0.0f, -10.0f));

    //Create "Floor"

	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(500.),btScalar(500.),btScalar(1.0f)));

	mCollisionShapesArray.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

	{
		btScalar mass(0.);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.0f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		mDynamicsWorld->addRigidBody(body);
	}
}

PhysicsManager::~PhysicsManager()
{
    //remove the rigidbodies from the dynamics world and delete them
	for (int i=mDynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = mDynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		mDynamicsWorld->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<mCollisionShapesArray.size();j++)
	{
	    if(mCollisionShapesArray[j] != 0)
	    {
	        btCollisionShape* shape = mCollisionShapesArray[j];
            mCollisionShapesArray[j] = 0;
            delete shape;
	    }
	}

	//delete dynamics world
	delete mDynamicsWorld;

	//delete solver
	delete mSolver;

	//delete broadphase
	delete mBroadphaseInterface;

	//delete dispatcher
	delete mDispatcher;

	delete mCollisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	mCollisionShapesArray.clear();
}

void PhysicsManager::simulate(double deltaTime)
{
    if(mPhysics) mDynamicsWorld->stepSimulation(deltaTime, 7);
}

unsigned int PhysicsManager::addRigidBody(btRigidBody* rigidbody, btCollisionShape* shape)
{
    mDynamicsWorld->addRigidBody(rigidbody);
    mCollisionShapesArray.push_back(shape);
    return mCollisionShapesArray.size()-1;
}

void PhysicsManager::removeRigidBody(btRigidBody* rigidbody, unsigned int shapeIndex)
{
    btCollisionShape* shape = mCollisionShapesArray[shapeIndex];
    if(shape != 0)
    {
        mCollisionShapesArray[shapeIndex] = 0;
        delete shape;
    }

    if(rigidbody && rigidbody->getMotionState())
    {
        delete rigidbody->getMotionState();
    }
    mDynamicsWorld->removeRigidBody(rigidbody);
    delete rigidbody;
}

void PhysicsManager::tickCallback(btDynamicsWorld *world, btScalar timeStep)
{
    int numManifolds = mDynamicsWorld->getDispatcher()->getNumManifolds();
    mCollisionSets.clear();

    //Iterate over all contact manifolds.
	for (int i = 0; i < numManifolds; i++)
	{
		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		const btRigidBody* rbA = btRigidBody::upcast(obA);
		const btRigidBody* rbB = btRigidBody::upcast(obB);


        if(rbA != 0 && rbB != 0)
        {
            MatterNode* matterA = (MatterNode*)obA->getUserPointer();
            MatterNode* matterB = (MatterNode*)obB->getUserPointer();

            if(matterA != 0) matterA->getMatter()->setCollided(true);
            if(matterB != 0) matterB->getMatter()->setCollided(true);

            if(matterA != 0 && matterB != 0)
            {
                int numContacts = contactManifold->getNumContacts();
                for (int j=0;j<numContacts;j++)
                {
                    btManifoldPoint& pt = contactManifold->getContactPoint(j);
                    if (pt.getDistance()<0.f)
                    {
                        btVector3 velA = rbA->getLinearVelocity();
                        btVector3 velB = rbB->getLinearVelocity();
//mPhysics = false;
                        //For some reason z is the wrong way around.
                        velA.setZ( -velA.z());
                        velB.setZ( -velB.z());

                        //Build collision object.
                        MatterCollision collision(matterA, matterB, velA, velB, pt, timeStep);

                        //Offer collision object to collision sets.
                        //Merge sets if necessary.
                        int offerResult = 0;
                        int index = -1;
                        for(int k = 0; k < mCollisionSets.size() && offerResult != -1; k++)
                        {
                            offerResult = mCollisionSets[k].offer(collision);
                            if(index == -1 && offerResult == 1)
                            {
                                index = k;
                                mCollisionSets[k].add(collision);
                            }
                            else if(index != -1 && offerResult == 1)
                            {
                                mCollisionSets[index].merge(mCollisionSets[k]);
                            }
                        }
                        //If did not find a home for the colllision, and it did not have duplicates, create a new set for it.
                        if(index == -1 && offerResult != -1)
                        {
                            mCollisionSets.push_back(MatterCollisionSet());
                            mCollisionSets.back().add(collision);
                        }

                    }
                }
            }
        }
	}

	for(unsigned int i = 0; i < mCollisionSets.size(); i++)
	{
	    mApp->getDestructionEngine()->processSet(mCollisionSets[i]);
	}

	mApp->getDestructionEngine()->checkForSeparation();

	std::cout << "END TICK" << std::endl;
}
