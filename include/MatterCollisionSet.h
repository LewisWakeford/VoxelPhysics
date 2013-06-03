#ifndef MATTERCOLLISIONSET_H
#define MATTERCOLLISIONSET_H

#include <vector>

#include "MatterCollision.h"

/*
    Class: MatterCollisionSet
    Stores a set of matter objects that have collided in the current physics tick.
*/
class MatterCollisionSet
{
    public:
        MatterCollisionSet();
        virtual ~MatterCollisionSet();

        //Add a collision to the set if a duplicate does not exist.
        //A duplicate is a collision with the participating matter objects.
        void offer(MatterCollision collision);
        void add(MatterCollision collision);

        unsigned int numCollisions() const;
        MatterCollision get(unsigned int index) const;

    protected:
        std::vector<MatterCollision> mMatterSet;



    private:
};

#endif // MATTERCOLLISIONSET_H
