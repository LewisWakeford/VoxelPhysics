#ifndef DESTRUCTIONENGINE_H
#define DESTRUCTIONENGINE_H

#include "MatterCollisionSet.h"
#include "EnergyGrid.h"

class App;

/*
    Class: DestructionEngine
    Manages desturction
*/
class DestructionEngine
{
    public:
        DestructionEngine(App* app);
        virtual ~DestructionEngine();

        void processSet(const MatterCollisionSet& set);
        void processCollision(const MatterCollision& collision);
        void buildBridges(unsigned int firstIndex, unsigned int secondIndex, const MatterCollision& collision);
        void transferEnergy(EnergyGrid& first, EnergyGrid& second);

        void checkForSeparation();

        unsigned int getGridIndex(MatterNode* matter);

    protected:

        std::vector<EnergyGrid> mEnergyGrids;

        App* mApp;

        int mNumberOfNonTrivialCollisions;
        int mDestructionsPerformed;

    private:
};

#endif // DESTRUCTIONENGINE_H
