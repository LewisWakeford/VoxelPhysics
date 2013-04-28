#include "RigidBody.h"

#include "App.h"
#include "PhysicsManager.h"
#include "Matter.h"
#include "VoxelField.h"
#include "MatterMotionState.h"

#include "console.h"

RigidBody::RigidBody(App* app)
{
    mApp = app;
    mRigidBody = 0;
    mShapeIndex = 0;
    mTransformOutdated = true;
    mInverseTransformOutdated = true;
}

RigidBody::~RigidBody()
{
    if(mRigidBody)
    {
        mApp->getPhysicsManager()->removeRigidBody(mRigidBody, mShapeIndex);
    }
}

void RigidBody::setProperties(MatterNode* matter, const std::vector<btConvexHullShape*>& hulls, float mass, Vector3 offset, Vector3 cOfM)
{
    if(mRigidBody)
    {
        mApp->getPhysicsManager()->removeRigidBody(mRigidBody, mShapeIndex);
    }

    btCompoundShape* rbShape = new btCompoundShape();

    //Attach clusters (Possible memory leak with children not being deleted?)
    for(int i = 0; i < hulls.size(); i++)
    {
        btConvexHullShape* chShape = hulls[i];

        btTransform chTransform; //Child local trans
        chTransform.setIdentity();
        //chTransform.setOrigin(btVector3(cOfM.x, cOfM.y, cOfM.z));

        rbShape->addChildShape(chTransform, chShape);
    }

    //Attach the actual Rigid body
    btTransform rbTransform;
	rbTransform.setIdentity();
	rbTransform.setOrigin(btVector3(offset.x, offset.y, offset.z));


    btScalar rbMass(mass);

    bool isDynamic = (rbMass != 0.0f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        rbShape->calculateLocalInertia(rbMass,localInertia);

    btDefaultMotionState* rbMotionState = new MatterMotionState(rbTransform); //Remember to implement own motion state.
    btRigidBody::btRigidBodyConstructionInfo rbInfo(rbMass,rbMotionState, rbShape,localInertia);
    rbInfo.m_restitution = 0.5f;
    rbInfo.m_friction = 2.0f;
    btRigidBody* body = new btRigidBody(rbInfo);

    body->setUserPointer(matter);

    mRigidBody = body;

    mShapeIndex = mApp->getPhysicsManager()->addRigidBody(mRigidBody, rbShape);
}

void RigidBody::setTransform(Matrix4D transform)
{
    /*
    btMatrix3x3 rbBasis(xx)
    btTransform rbTransform;
    rbTransform.
    */
}

const Matrix4D& RigidBody::getTransform()
{
    if(mRigidBody)
    {
        MatterMotionState* mtst = (MatterMotionState*)mRigidBody->getMotionState();
        mTransformOutdated = mtst->mTransformOutdated;
        if(mTransformOutdated)
        {
            mTransform = Matrix4D::createIdentity();

            btTransform rbTransform;
            mtst->getWorldTransform(rbTransform);
            btVector3 rbOrigin = rbTransform.getOrigin();
            btMatrix3x3 rbBasis = rbTransform.getBasis();

            btVector3 row1 = rbBasis.getRow(0);
            btVector3 row2 = rbBasis.getRow(1);
            btVector3 row3 = rbBasis.getRow(2);

            mTransform.setRow(0, row1.x(), row1.y(), row1.z(), rbOrigin.x());
            mTransform.setRow(1, row2.x(), row2.y(), row2.z(), rbOrigin.y());
            mTransform.setRow(2, row3.x(), row3.y(), row3.z(), rbOrigin.z());
            mTransform.setRow(3, 0, 0, 0, 1);

            mTransformOutdated = false;
            mInverseTransformOutdated = true;

            mtst->mTransformOutdated = false;

        }
        return mTransform;
    }
    else
    {
        consolePrint("NO RIGID BODY");
        return Matrix4D::createIdentity();
    }
}

const Matrix4D& RigidBody::getInvertedTransform()
{
    if(mRigidBody)
    {
        if(mInverseTransformOutdated)
        {
            mInvertedTransform = getTransform().inverted();

            mInverseTransformOutdated = false;
        }
        return mInvertedTransform;
    }
    else
    {
        consolePrint("NO RIGID BODY");
        return Matrix4D::createIdentity();
    }
}
