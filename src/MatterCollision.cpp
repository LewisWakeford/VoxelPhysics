#include "MatterCollision.h"

MatterCollision::MatterCollision(MatterNode* first, MatterNode* second, const btVector3& velocityOnFirst, const btVector3& velocityOnSecond, btManifoldPoint point, double timeStep)
{
    mFirst = first;
    mSecond = second;
    mVelocityFirst = velocityOnFirst;
    mVelocitySecond = velocityOnSecond;
    mManifoldPoint = point;
    mTimeStep = timeStep;
}

MatterCollision::~MatterCollision()
{
    //dtor
}

MatterNode* MatterCollision::getFirst() const
{
    return mFirst;
}

MatterNode* MatterCollision::getSecond() const
{
    return mSecond;
}

const btManifoldPoint& MatterCollision::getPoint() const
{
    return mManifoldPoint;
}

const btVector3& MatterCollision::getNormalOnFirst() const
{
    return -mManifoldPoint.m_normalWorldOnB;
}
const btVector3& MatterCollision::getNormalOnSecond() const
{
    return mManifoldPoint.m_normalWorldOnB;
}

const btVector3& MatterCollision::getVelocityFirst() const
{
    return mVelocityFirst;
}

const btVector3& MatterCollision::getVelocitySecond() const
{
    return mVelocitySecond;
}
