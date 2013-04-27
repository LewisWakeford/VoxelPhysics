#ifndef MATTERCOLLISIONSET_H
#define MATTERCOLLISIONSET_H

#include <vector>

#include "MatterCollision.h"

/*
    Class: MatterCollisionSet
    Stores a set of matter objects that have collided in the current physics tick.
    Basically if A collides with B then they are put in a set (obviously) but if C also collides with B then it shares one set ABC instead of starting a new one.
    The offer() function takes a matter node pair and returns true if one of them is already in the set, while adding the other.
    If two or more of the physics manager's sets return true then they are merged with with the merge() function.
    By storing a few of these collision sets the physics manager can quickly group the matter nodes into sets ready for processing by the destuction engine.
    The destruction engine takes collision sets as input.
*/
class MatterCollisionSet
{
    public:
        MatterCollisionSet();
        virtual ~MatterCollisionSet();

        //Returns -1 if the collision should be discarded (IE duplicate exists), 0 if collision is not accepted and 1 if this set will accept the collision.
        int offer(MatterCollision collision);
        void merge(MatterCollisionSet& other);
        void add(MatterCollision collision);

        unsigned int numCollisions() const;
        MatterCollision get(unsigned int index) const;
    protected:
        bool mValid;
        std::vector<MatterCollision> mMatterSet;



    private:
};

#endif // MATTERCOLLISIONSET_H
