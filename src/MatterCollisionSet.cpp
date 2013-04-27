#include "MatterCollisionSet.h"
#include "Matter.h"
#include "MatterNode.h"

MatterCollisionSet::MatterCollisionSet()
{
    mValid = true;
}

MatterCollisionSet::~MatterCollisionSet()
{
    //dtor
}


//Return true if this collision belongs in this set.
int MatterCollisionSet::offer(MatterCollision collision)
{
    if(!mValid) return false;

    bool firstInSet = false;
    bool secondInSet = false;
    for(int i = 0; (!firstInSet || !secondInSet) && i < mMatterSet.size(); i++)
    {
        if(!firstInSet && (collision.getFirst()->getMatter()->getID() == mMatterSet[i].getFirst()->getMatter()->getID() ||
                           collision.getFirst()->getMatter()->getID() == mMatterSet[i].getSecond()->getMatter()->getID()))
        {
            firstInSet = true;
        }
        if(!secondInSet && (collision.getSecond()->getMatter()->getID() == mMatterSet[i].getFirst()->getMatter()->getID() ||
                            collision.getSecond()->getMatter()->getID() == mMatterSet[i].getSecond()->getMatter()->getID()))
        {
            secondInSet = true;
        }
    }
    if(firstInSet && secondInSet)
    {
        return -1;
    }
    else if(firstInSet || secondInSet)
    {
        return 1;
    }
    else return 0;
}

void MatterCollisionSet::merge(MatterCollisionSet& other)
{
    const std::vector<MatterCollision>& otherSet = other.mMatterSet;
    for(int i = 0; i < otherSet.size(); i++)
    {
        mMatterSet.push_back(otherSet[i]);
    }
    other.mValid = false;
}

void MatterCollisionSet::add(MatterCollision collision)
{
    mMatterSet.push_back(collision);
}

unsigned int MatterCollisionSet::numCollisions() const
{
    return mMatterSet.size();
}

MatterCollision MatterCollisionSet::get(unsigned int index) const
{
    return mMatterSet[index];
}
