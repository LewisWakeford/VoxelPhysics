#ifndef DESTRUCTIONENGINE_H
#define DESTRUCTIONENGINE_H

#include "MatterCollisionSet.h"
#include "EnergyGrid.h"

class App;

/*
    Class: DestructionEngine
    Manages destruction of Matter Objects
*/
class DestructionEngine
{
    public:
        DestructionEngine(App* app);
        virtual ~DestructionEngine();

        //Process a set of collisions between matter objects.
        void processSet(const MatterCollisionSet& set);

        //Process an individual collision between matter objects.
        void processCollision(const MatterCollision& collision);

        //Find "bridge" voxels between two matter objects.
        void buildBridges(unsigned int firstIndex, unsigned int secondIndex, const MatterCollision& collision);

        //Perform energy transfer between two matter objects.
        void transferEnergy(EnergyGrid& first, EnergyGrid& second);

        //Check for breaking in all matter objects that collided this tick.
        void checkForSeparation();

        //Find the index for the EnergyGrid that represents the given MatterNode.
        unsigned int getGridIndex(MatterNode* matter);

    protected:

        //Array of Energy Grids being used in this tick
        std::vector<EnergyGrid> mEnergyGrids;

        App* mApp;

        //Used for debugging
        int mNumberOfNonTrivialCollisions;
        int mDestructionsPerformed;

    private:
};

#endif // DESTRUCTIONENGINE_H
