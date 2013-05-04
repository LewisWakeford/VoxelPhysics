#include "MatterCollisionSet.h"
#include "Matter.h"
#include "MatterNode.h"

MatterCollisionSet::MatterCollisionSet()
{

}

MatterCollisionSet::~MatterCollisionSet()
{
    //dtor
}


//Return true if this collision belongs in this set.
void MatterCollisionSet::offer(MatterCollision collision)
{
    bool found = false;

    for(int i = 0; !found && i < mMatterSet.size(); i++)
    {
        bool first = false;
        bool second = false;
        if(collision.getFirst()->getMatter()->getID() == mMatterSet[i].getFirst()->getMatter()->getID() ||
                           collision.getFirst()->getMatter()->getID() == mMatterSet[i].getSecond()->getMatter()->getID())
        {
            first = true;
        }
        if(collision.getSecond()->getMatter()->getID() == mMatterSet[i].getFirst()->getMatter()->getID() ||
                            collision.getSecond()->getMatter()->getID() == mMatterSet[i].getSecond()->getMatter()->getID())
        {
            second = true;
        }
        if(first && second)
        {
            found = true;
        }
    }

    if(!found)
    {
        add(collision);
    }
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
