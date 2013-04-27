#ifndef TRANSFERGRAPH_H
#define TRANSFERGRAPH_H

#include "EnergyGrid.h"

/*
    Class TransferGraph
    A transfer graph is used during the "indirect transfer" phase of energy transfer.
    Every voxel non-energised voxel is given several "feeder" voxels.
    To energise a voxel, the energy is requested evenly from all feeders, which in turn request it from their feeders and so on.
    Each request generates stress and pressure along the chain.
*/
class EnergyGrid::TransferGraph
{
    public:
        TransferGraph(std::vector<Vector3i> startingVoxels);
        virtual ~TransferGraph();

    protected:
        Voxel

    private:

};

#endif // TRANSFERGRAPH_H
