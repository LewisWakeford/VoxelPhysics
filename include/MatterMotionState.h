#ifndef MATTERMOTIONSTATE_H
#define MATTERMOTIONSTATE_H

#include "Matter.h"

class MatterMotionState : public btDefaultMotionState
{
    public:
        MatterMotionState(const btTransform& startTrans = btTransform::getIdentity()) : btDefaultMotionState(startTrans), mTransformOutdated(true)
        {

        };
        virtual ~MatterMotionState()
        {

        };

        inline void setWorldTransform(const btTransform& trans)
        {
            btDefaultMotionState::setWorldTransform(trans);
            mTransformOutdated = true;
        }

        bool mTransformOutdated;

    protected:

    private:
};

#endif // MATTERMOTIONSTATE_H
