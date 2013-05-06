#include "EnergyGrid.h"

#include "Matter.h"
#include "MatterNode.h"

std::mt19937 generator;
std::uniform_int_distribution<int> distribution(0, 100);
auto rand100 = std::bind ( distribution, generator );

EnergyGrid::eDirection EnergyGrid::DIRECTION_START = DOWN;

int EnergyGrid::sDirectionAsBit[27] =
        {
            //DOWN
                0x1,
            //UP
                0x2,
            //LEFT
                0x4,
            //RIGHT
                0x8,
            //FRONT
                0x10,
            //BACK
                0x20,
            //LEFT_FRONT
                0x40,
            //LEFT_BACK
                0x80,
            //RIGHT_FRONT
                0x100,
            //RIGHT_BACK
                0x200,
            //UP_LEFT
                0x400,
            //UP_RIGHT
                0x800,
            //UP_FRONT
                0x1000,
            //UP_BACK
                0x2000,
            //UP_LEFT_FRONT
                0x4000,
            //UP_LEFT_BACK
                0x8000,
            //UP_RIGHT_FRONT
                0x10000,
            //UP_RIGHT_BACK
                0x20000,
            //DOWN_LEFT
                0x40000,
            //DOWN_RIGHT
                0x80000,
            //DOWN_FRONT
                0x100000,
            //DOWN_BACK
                0x200000,
            //DOWN_LEFT_FRONT
                0x400000,
            //DOWN_LEFT_BACK
                0x800000,
            //DOWN_RIGHT_FRONT
                0x1000000,
            //DOWN_RIGHT_BACK
                0x2000000,
            //INVALID_DIR
                0x0
        };

//Pre-Computed.
int
 EnergyGrid::sAdjanctDirections[27] =
{50070525,262142,16053495,37436219,6118751,45002415,5526615,10790055,1644827,36317739,62710,211770,89438,175790,21590,42150,71962,141866,15991029,37225273,6029661,44827309,5505109,10748069,1573145,36176425};


EnergyGrid::EnergyGrid(MatterNode* matterNode) :
    mTransferMap(32, std::vector<std::vector<int>>(32, std::vector<int>(32, -1))),
    mVoxelData(32, std::vector<std::vector<VoxelData>>(32, std::vector<VoxelData>(32)))
{
    mMatterNode = matterNode;

    for(int z = 0; z < 32; z++)
    {
        for(int y = 0; y < 32; y++)
        {
            for(int x = 0; x < 32; x++)
            {
                if(getMatter()->getVoxelField()->get(x, y, z) == 1)
                {
                    mVoxelData[x][y][z].mFull = true;
                    mVoxelData[x][y][z].mStrength = getStrength(x, y, z);
                }
            }
        }
    }
    mDestructionOccured = false;
    mSnappingOccured = false;

    bool mFirstExternalTransfer = true;
    bool mFirstInternalTransfer = true;
}

EnergyGrid::~EnergyGrid()
{
    //dtor
}

float EnergyGrid::getStrength(int x, int y, int z)
{
    int adjanctVoxels = 0;
    Vector3i coord(x,y,z);
    for(int i = DIRECTION_START; i < INVALID_DIR; i++)
    {
        const Vector3i& directionVector = getDirectionVector(i);
        Vector3i adjanctCoord = coord + directionVector;
        if(getMatter()->getVoxelField()->get(adjanctCoord.x, adjanctCoord.y, adjanctCoord.z) == 1) adjanctVoxels++;
    }
    bool random = rand100() > getMatter()->getMaterial()->getRandomChance()*100;
    float strength = 1.0f;
    if(random)
    {
        strength = (float(rand100())/100.0f*(getMatter()->getMaterial()->getMaxStrength() - getMatter()->getMaterial()->getMinStrength()))
                    + getMatter()->getMaterial()->getMinStrength();
    }
    return strength;
}

void EnergyGrid::setEnergy(const Vector3f& energy)
{
    mEnergyVector = energy;
    mEnergyVectorLocal = getMatter()->getRigidBody()->getInvertedTransform().rotateVector(energy);
    mFirstInternalTransfer = true;
}

Vector3f EnergyGrid::getEnergyVector()
{
    return mEnergyVector;
}

void EnergyGrid::setCollisionPartner(EnergyGrid* energyGrid)
{
    mCollisionPartner = energyGrid;
    mFirstExternalTransfer = true;
}

void EnergyGrid::buildMaps(const Vector3f& otherEnergy)
{
    //Get the direction of the energy.
    Vector3f myDirection = mEnergyVectorLocal.normalized();
    Vector3f otherDirection = otherEnergy.normalized();

    //Transform the other direction into voxel space.
    otherDirection = getMatter()->getRigidBody()->getTransform().inverted().rotateVector(otherDirection);

    //Build Maps
    mProjectorMap.resize(26);
    mRecieverMap.resize(26);

    //Projector.
    float totalPressure = 0.0f;
    for(unsigned int i = DIRECTION_START; i < INVALID_DIR; ++i)
    {
        Vector3f directionVector = getDirectionVectorF(i);
        float pressure = directionVector.dot(myDirection);
        mProjectorMap[i] = pressure;

        if(pressure > 0) totalPressure += pressure;

        mRecieverMap[i] = directionVector.dot(myDirection);
    }

    for(unsigned int i = 0; i < 26; i++)
    {
        if(mProjectorMap[i] > 0.0f)
        {
            mPressureDirectionsP.push_back(i);
            mDirectTransferMap[i] = mProjectorMap[i] / totalPressure;
        }
        if(mRecieverMap[i] > 0.0f) mPressureDirectionsR.push_back(i);
    }

    //Create stress priority directions
    std::vector<float> tempProjectorMap = mProjectorMap;

    for(char i = 0; i < 26; i++)
    {
        float lowestPressure = 2.0f;
        char lowestIndex = -1;
        for(char k = 0; k < 26; k++)
        {
            float pressureRatio = fabs(tempProjectorMap[k]);
            if(pressureRatio < lowestPressure)
            {
                lowestPressure = pressureRatio;
                lowestIndex = k;
            }
        }
        mStressDirectionsP[lowestIndex] = i;
        tempProjectorMap[lowestIndex] = 2.0f;
    }

    std::vector<float> tempRecieverMap = mRecieverMap;

    for(char i = 0; i < 26; i++)
    {
        float lowestPressure = 2.0f;
        char lowestIndex = -1;
        for(char k = 0; k < 26; k++)
        {
            float pressureRatio = fabs(tempRecieverMap[k]);
            if(pressureRatio < lowestPressure)
            {
                lowestPressure = pressureRatio;
                lowestIndex = k;
            }
        }
        mStressDirectionsR[lowestIndex] = i;
        tempRecieverMap[lowestIndex] = 2.0f;
    }
}

void EnergyGrid::setAsProjector()
{
    mCurrentMap = &mProjectorMap;
    mCurrentPressureDirections = &mPressureDirectionsP;
    mCurrentStressDirections = &mStressDirectionsP;
    mIsReciever = false;
}

void EnergyGrid::setAsReciever()
{
    mCurrentMap = &mRecieverMap;
    mCurrentPressureDirections = &mPressureDirectionsR;
    mCurrentStressDirections = &mStressDirectionsR;
    mIsReciever = true;
}

void EnergyGrid::setInitialEnergy(float energy)
{
    unsigned int totalVoxels = getMatter()->getVoxelField()->getNumVoxels();
    float energyPerVoxel = energy / totalVoxels;

    mStartingEnergyPerVoxel = energyPerVoxel;

    for(int z = 0; z < 32; z++)
    {
        for(int y = 0; y < 32; y++)
        {
            for(int x = 0; x < 32; x++)
            {
                if(mVoxelData[x][y][z].mFull)
                {
                    mVoxelData[x][y][z].setEnergy(mIsReciever, energyPerVoxel);
                }
            }
        }
    }
}

void EnergyGrid::directTransfer()
{
    for(auto i = mBridges.begin(); i != mBridges.end(); i++)
    {
        transferInternalEnergyThroughBridge(i);
    }
}

//There is an issue I need to solve with indirect transfer:
//The problem is that energy might "fall off" but all the voxels in the matter are already energised.
//I need to rethink this bit...

/*
    The current method should work for all recieving matters? If they have less energy than the projector, yes.
    What about otherwise? Meh, should be okay.
    So the issue is only when a projector is a weird shape and not all of it's energy can transfer via the bridges.
    It may be worth having a second phase for this then... Basically nodes with left over energy need to do indirect transfer until they get to a bridge.
    So build a seperate transfer graph going from nodes with excess energy in every direction until they hit some bridge(s).
    At that point do pullEnergy on each bridge node. Then project the same amount of energy along the bridge(s).

    ACTUALLY: Go the other way, start with bridges and then expand until you find overflowing nodes, then suck energy back.

    That's a bit innaccurate still, so maybe something simpler when doing the direct transfer, IE as soon as energy falls off it gets spread somewhere else.

    Nah. Start from bridges and work backwards. However, if any snappage/breakage occurs, the graph may need fixing. This is a feature that should be in the regular indirect transfer too.
    Just store the "generation" of a node as the graph is built, when a node ends up with no feeders, it checks the adjanct nodes. The ones with the joint lowest generation are new feeders.
    If all are broken then the isolation of the node is fine and that's how it should be.

    Also need a way to recover energy if a voxel snaps. Maybe just recover it all? Yeah lets do that.

    NEW VERSION: Only 1 feeder per voxel, the feeder from the most "stressful" direction is the one used.
*/
void EnergyGrid::indirectTransfer()
{
    double startOfTransfer = glfwGetTime();
    buildTransferGraph();
    double buildGraph = glfwGetTime();
    float energyToTransfer = fabs(mEnergyPerVoxel - mStartingEnergyPerVoxel);
    //std::cout << "Time to build graph: " << (buildGraph - startOfTransfer) << std::endl;
    for(unsigned int i = 0; i < mTransferGraph.size(); i++)
    {
        TransferNode& currentNode = mTransferGraph[i];
        if(!currentNode.mSource)
        {
            Vector3i coord = currentNode.mVoxelCoord;
            VoxelData& voxel = mVoxelData[coord.x][coord.y][coord.z];
            stressVoxel(coord, currentNode.mAccumulatedStress*energyToTransfer);
            /* Old Code
            Vector3i coord = currentNode.mVoxelCoord;
            VoxelData& voxel = mVoxelData[coord.x][coord.y][coord.z];

            float requiredEnergy = mEnergyPerVoxel - voxel.getEnergy(mIsReciever);
            if(requiredEnergy > 0) pullEnergy(i, requiredEnergy, INVALID_DIR);
            voxel.setEnergy(mIsReciever, mEnergyPerVoxel);
            */
        }
    }
    double transfer = glfwGetTime();
    std::cout << "Time to transfer: " << (transfer - buildGraph) << std::endl;
}

void EnergyGrid::transferInternalEnergyThroughBridge(std::unordered_map<int, int>::const_iterator bridgeIterator)
{
    double internalStart = glfwGetTime();
    int key = bridgeIterator->first;
    Vector3i bridgePoint = getBridgeVector(key);

    //Quick bit of setup
    unsigned int x = 0;
    unsigned int y = 1;
    unsigned int z = 2;

    const Vector3f internalDelta = getInternalTransferDelta();

    //Keep a track record of voxels weights and such
    std::vector<Vector3i> voxelsInPath;
    std::vector<int> voxelDistance;

    //Find where the energy vector intersects each of the 32 planes in the significant direction.
    //At each intersection find the four closest voxels in that plane to the intersection point.
    //Spread energy between them based on distance from point of intersection.

    //Bridges local coord is awlays grid aligned.
    Vector3f pointOfIntersection = Vector3f(bridgePoint.x, bridgePoint.y, bridgePoint.z);

    float energyPool = 0.0f;

    int maxDistance = 0;

    bool intersectValid = true;

    while(intersectValid)
    {
        //NOTE: We can assume that the line is not parrallel to the plane as we change the plane we intersect with based on direction of energy.


        //Equation of plane is sig = pointInPlane.sig

        //Equation of line is:
        // sig = pointCoord.sig + (partnerDirection.sig * t)
        // up = pointCoord.up + (partnerDirection.up * t)
        // left = pointCoord.left + (partnerDirection.left * t)

        //Recreate plane with t and solve for t
        // pointCoord.sig + (partnerDirection.sig * t) = pointInPlane.sig
        // partnerDirection.sig * t = pointInPlane.sig - pointCoord.sig
        // t = (pointInPlane.sig - pointCoord.sig) / partnerDirection.sig

        Vector3i closestVoxel;

        int upCoord = roundf(pointOfIntersection.get(mInternalTransferUp));
        int leftCoord = roundf(pointOfIntersection.get(mInternalTransferLeft));
        int sigCoord = roundf(pointOfIntersection.get(mInternalTransferSig));


        if(mInternalTransferSig == x)
        {
            closestVoxel = Vector3i(sigCoord, upCoord, leftCoord);
        }
        else if(mInternalTransferSig == y)
        {
            closestVoxel = Vector3i(upCoord, sigCoord, leftCoord);
        }
        else
        {
            closestVoxel = Vector3i(leftCoord, upCoord, sigCoord);
        }

        //Add valid voxel to path.

        if(validCoord(closestVoxel))
        {
            VoxelData& voxel = mVoxelData[closestVoxel.x][closestVoxel.y][closestVoxel.z];
            float freeEnergyInVoxel = (voxel.getEnergy(mIsReciever)-mEnergyPerVoxel);
            energyPool += freeEnergyInVoxel;
            voxel.setEnergy(mIsReciever, mEnergyPerVoxel);
            voxelsInPath.push_back(closestVoxel);
            voxelDistance.push_back(maxDistance);
        }
        else
        {
            intersectValid = false;
        }

        if(intersectValid)
        {
            maxDistance++;
            pointOfIntersection += internalDelta;
        }

    }

    //Apply pressure to voxels in path.
    for(unsigned int i = 0; i < voxelsInPath.size(); i++)
    {
        int pressuringLayers = 1 + ((maxDistance - voxelDistance[i])*2); //How many layers of voxels are applying pressure to this one.
        float pressure = pressuringLayers * (mStartingEnergyPerVoxel-mEnergyPerVoxel);
        pressureVoxel(voxelsInPath[i], pressure);
    }

    double internalDone = glfwGetTime();
    //std::cout << "Int Transfer: " << (internalDone - internalStart) << std::endl;

    Vector3i bridgePoint3i = getBridgeVector(bridgeIterator->second);
    mCollisionPartner->transferExternalEnergyTo(bridgePoint3i, energyPool);
    double externalDone = glfwGetTime();
    //std::cout << "Ext Transfer: " << (externalDone - internalDone) << std::endl;

}

void EnergyGrid::transferExternalEnergyTo(const Vector3i& bridgePoint, float energy)
{
    double internalStart = glfwGetTime();

    //Quick bit of setup
    unsigned int x = 0;
    unsigned int y = 1;
    unsigned int z = 2;

    const Vector3f externalDelta = getExternalTransferDelta();

    //Keep a track record of voxels weights and such
    std::vector<Vector3i> voxelsInPath;
    std::vector<int> voxelDistance;

    //Find where the energy vector intersects each of the 32 planes in the significant direction.
    //At each intersection find the four closest voxels in that plane to the intersection point.
    //Spread energy between them based on distance from point of intersection.

    //Bridges local coord is awlays grid aligned.
    Vector3f pointOfIntersection = Vector3f(bridgePoint.x, bridgePoint.y, bridgePoint.z);

    float energyAbsorbedPerVoxel = mEnergyPerVoxel - mStartingEnergyPerVoxel;

    int maxDistance = 0;

    bool intersectValid = true;

    while(intersectValid)
    {
        //NOTE: We can assume that the line is not parrallel to the plane as we change the plane we intersect with based on direction of energy.


        //Equation of plane is sig = pointInPlane.sig

        //Equation of line is:
        // sig = pointCoord.sig + (partnerDirection.sig * t)
        // up = pointCoord.up + (partnerDirection.up * t)
        // left = pointCoord.left + (partnerDirection.left * t)

        //Recreate plane with t and solve for t
        // pointCoord.sig + (partnerDirection.sig * t) = pointInPlane.sig
        // partnerDirection.sig * t = pointInPlane.sig - pointCoord.sig
        // t = (pointInPlane.sig - pointCoord.sig) / partnerDirection.sig

        Vector3i closestVoxel;

        int upCoord = roundf(pointOfIntersection.get(mExternalTransferUp));
        int leftCoord = roundf(pointOfIntersection.get(mExternalTransferLeft));
        int sigCoord = roundf(pointOfIntersection.get(mExternalTransferSig));


        if(mExternalTransferSig == x)
        {
            closestVoxel = Vector3i(sigCoord, upCoord, leftCoord);
        }
        else if(mExternalTransferSig == y)
        {
            closestVoxel = Vector3i(upCoord, sigCoord, leftCoord);
        }
        else
        {
            closestVoxel = Vector3i(leftCoord, upCoord, sigCoord);
        }

        //Add valid voxel to path.

        if(validCoord(closestVoxel))
        {
            VoxelData& voxel = mVoxelData[closestVoxel.x][closestVoxel.y][closestVoxel.z];
            float freeEnergyInVoxel = (voxel.getEnergy(mIsReciever)-mEnergyPerVoxel);
            voxel.setEnergy(mIsReciever, mEnergyPerVoxel);
            voxelsInPath.push_back(closestVoxel);
            voxelDistance.push_back(maxDistance);
        }
        else
        {
            intersectValid = false;
        }
        if(intersectValid)
        {
            maxDistance++;
            pointOfIntersection += externalDelta;
        }

    }

    //Apply pressure to voxels in path.
    for(unsigned int i = 0; i < voxelsInPath.size(); i++)
    {
        float pressure = (2*energy) - ((2 * voxelDistance[i] + 1) * energyAbsorbedPerVoxel);
        pressureVoxel(voxelsInPath[i], pressure);
    }
}

void EnergyGrid::buildTransferGraph()
{
    mTransferGraph.clear();

    unsigned int startOfStep = 0;
    unsigned int endOfStep = 0;

    //Reset the transfermap
    mTransferMap = std::vector<std::vector<std::vector<int>>>(32, std::vector<std::vector<int>>(32, std::vector<int>(32, -1)));

    float sourceLowerBound = mEnergyPerVoxel*0.9;
    float sourceUpperBound = mEnergyPerVoxel*1.1;

    //Add source voxels to graph
    for(unsigned int x = 0; x < 32; x++)
    {
        for(unsigned int y = 0; y < 32; y++)
        {
            for(unsigned int z = 0; z < 32; z++)
            {
                VoxelData& voxel = mVoxelData[x][y][z];
                float voxelEnergy = voxel.getEnergy(mIsReciever);
                if(voxel.mFull)
                {
                    if(voxelEnergy < sourceUpperBound && voxelEnergy > sourceLowerBound)
                    {
                        mTransferGraph.push_back(TransferNode(true, Vector3i(x, y, z)));
                        mTransferGraph.back().mGeneration = 0;
                        mTransferMap[x][y][z] = mTransferGraph.size()-1;
                        endOfStep++;
                    }
                }//&& !voxel.mDestroyed &&


            }
        }
    }

    unsigned int oldSize = 0;
    unsigned int newSize = endOfStep;

    unsigned int generation = 1;

    //Expand graph
    while(oldSize != newSize) //End if no new voxels are added to graph.
    {
        oldSize = newSize;
        endOfStep = newSize;

        //Add adjanct nodes to map.
        for(unsigned int i = startOfStep; i < endOfStep; i++)
        {
            TransferNode& currentNode = mTransferGraph[i];
            const Vector3i coord = currentNode.mVoxelCoord;
            int adjanctNodesToFeeder = sAdjanctDirections[currentNode.mFeederDirection];

            //Get coords of adjanct nodes
            for(unsigned int k = 0; k < 27; k++)
            {
                if(! (sDirectionAsBit[k] & adjanctNodesToFeeder))
                {
                    Vector3i adjanctCoord = coord + getDirectionVector(k);
                    if(validCoord(adjanctCoord))
                    {
                        int nodeIndex = mTransferMap[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];

                        //Node has already been added to graph, but is "younger" than this node.
                        if(nodeIndex >= 0 && nodeIndex > endOfStep)
                        {
                            TransferNode& adjanctNode = mTransferGraph[nodeIndex];
                            eDirection previousFeederDirection = getReverseDirection(adjanctNode.mFeederDirection);
                            char previousFeederPriority = mCurrentStressDirections->at(k);
                            char currentNodePriority = mCurrentStressDirections->at(k);

                            //If this voxel is a better feeder, then use it instead of the previous one.
                            if(currentNodePriority < previousFeederPriority)
                            {
                                adjanctNode.mFeeder = i;
                                adjanctNode.mFeederDirection = getReverseDirection(k);
                            }
                        }
                        else if(nodeIndex < 0)
                        {
                            //Node has not been added, must check it is full.
                            VoxelData& voxel = mVoxelData[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];
                            if(voxel.mFull// && !(voxel.mDestroyed || voxel.mSnapped)
                               )
                            {
                                int insertedIndex = mTransferGraph.size();
                                mTransferGraph.push_back(TransferNode(false, adjanctCoord));
                                mTransferMap[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z] = insertedIndex;
                                TransferNode& newNode = mTransferGraph.back();
                                newNode.mFeeder = i;
                                newNode.mFeederDirection = getReverseDirection(k);
                                newNode.mGeneration = generation;
                                stressTransferNode(insertedIndex, 1.0f);
                            }
                        }
                    }
                }
            }
        }

        newSize = mTransferGraph.size();
        startOfStep = endOfStep;
        generation++;
    }

}

void EnergyGrid::stressTransferNode(int index, float stress)
{
    if(index > -1)
    {
        mTransferGraph[index].mAccumulatedStress += stress;
        stressTransferNode(mTransferGraph[index].mFeeder, stress);
    }
}

void EnergyGrid::directTransferVoxel(Vector3i sourceVoxel)
{
    /*
    if(validCoord(sourceVoxel))
    {
        VoxelData& voxel = mVoxelData[sourceVoxel.x][sourceVoxel.y][sourceVoxel.z];

        float energyToTransfer = voxel.getEnergy(mIsReciever) - mEnergyPerVoxel;

        //For each voxel that lies "infront" of this one in terms of the energy vector...
        for(unsigned int i = 0; i < mCurrentPressureDirections->size(); i++)
        {
            //Transfer a portion of the free energy.
            char direction = mCurrentPressureDirections->at(i);
            Vector3i directionVector = getDirectionVector(direction);
            Vector3i targetCoord = directionVector + sourceVoxel;

            float transferredEnergy = mDirectTransferMap[direction] * energyToTransfer;
            pressureVoxel(sourceVoxel, transferredEnergy);
            transferEnergyTo(targetCoord, direction, transferredEnergy);
        }

        voxel.setEnergy(mIsReciever, mEnergyPerVoxel);
    }
    */
}

/*
void EnergyGrid::transferEnergy(Vector3i sourceVoxel, char direction, float energy)
{
    float transferredEnergy = transferEnergyFrom(sourceVoxel, direction, energy);
    Vector3i directionVector = getDirectionVector(direction);
    Vector3i targetCoord = directionVector + sourceVoxel;
    transferEnergyTo(targetCoord, direction, transferredEnergy);
}
*/

//Used in external transfer, sucks out some free energy
float EnergyGrid::transferEnergyFrom(Vector3i sourceVoxel, float energy)
{
    if(validCoord(sourceVoxel))
    {
        VoxelData& source = mVoxelData[sourceVoxel.x][sourceVoxel.y][sourceVoxel.z];

        float energyToTransfer = source.getEnergy(mIsReciever) - mEnergyPerVoxel;

        //Take all energy
        if(energy >= energyToTransfer && energyToTransfer >= 0)
        {
            pressureVoxel(sourceVoxel, energyToTransfer);
            source.setEnergy(mIsReciever, mEnergyPerVoxel);
            return energyToTransfer;
        }
        //Only take some
        else if(energyToTransfer >= 0)
        {
            pressureVoxel(sourceVoxel, energy);
            source.setEnergy(mIsReciever, energyToTransfer-energy+mEnergyPerVoxel);
            return energy;
        }
    }
    return 0.0f;
}

//Returns the key to the bridge.
//Which is the x-y-z coords compressed into an int.
int EnergyGrid::getBridgeKey(const Vector3i& voxelCoord)
{
    //Max value, 33 so 6 bit ints.
    return voxelCoord.x << 12 | voxelCoord.y << 6 | voxelCoord.z;
}

bool EnergyGrid::getBridge(int key, int& ret_externalPoint)
{
    std::unordered_map<int, int>::const_iterator it = mBridges.find(key);
    if(it == mBridges.end() )
    {
        return false;
    }
    else
    {
        ret_externalPoint = it->second;
        return true;
    }
}

Vector3i EnergyGrid::getBridgeVector(int key)
{
    int x = key >> 12;
    int y = key >> 6 & 63; //Mask with 111111 which is 63
    int z = key & 63;
    return Vector3i(x, y, z);
}

void EnergyGrid::transferEnergyTo(Vector3i targetVoxel, char direction, float energy)
{
    /*int bridgeKey = getBridgeKey(targetVoxel);
    int bridgePoint;
    bool isBridge = getBridge(bridgeKey, bridgePoint);

    if(isBridge)
    {
        mCollisionPartner->transferExternalEnergyTo(bridgePoint, energy);
    }
    else
    {
        if(validCoord(targetVoxel))
        {
            VoxelData& target = mVoxelData[targetVoxel.x][targetVoxel.y][targetVoxel.z];
            if(!target.mFull)
            {
    //            mEnergyNotAbsorbed += energy;
            }
            else
            {
                //float pressureFactor = mCurrentMap[direction];
                //float stressFactor = 1.0f - pressureFactor;
                target.addEnergy(mIsReciever, energy);
                pressureVoxel(targetVoxel, energy);
                //stressVoxel(targetVoxel, stressFactor * energy);
            }
        }
    }
    */
}

void EnergyGrid::directTransferTo(Vector3i targetVoxel, float energy)
{
    if(validCoord(targetVoxel))
    {
        VoxelData& voxel = mVoxelData[targetVoxel.x][targetVoxel.y][targetVoxel.z];

        voxel.addEnergy(mIsReciever, energy);
        pressureVoxel(targetVoxel, energy);
    }
}



EnergyGrid::eDirection EnergyGrid::getReverseDirection(char direction)
{
    switch(direction)
    {
        case UP:
            return DOWN;
        break;
        case DOWN:
            return UP;
        break;
        case LEFT:
            return RIGHT;
        break;
        case RIGHT:
            return LEFT;
        break;
        case FRONT:
            return BACK;
        break;
        case BACK:
            return FRONT;
        break;
        case LEFT_FRONT:
            return RIGHT_BACK;
        break;
        case LEFT_BACK:
            return RIGHT_FRONT;
        break;
        case RIGHT_FRONT:
            return LEFT_BACK;
        break;
        case RIGHT_BACK:
            return LEFT_FRONT;
        break;
        case UP_LEFT:
            return DOWN_RIGHT;
        break;
        case UP_RIGHT:
            return DOWN_LEFT;
        break;
        case UP_FRONT:
            return DOWN_BACK;
        break;
        case UP_BACK:
            return DOWN_FRONT;
        break;
        case UP_LEFT_FRONT:
            return DOWN_RIGHT_BACK;
        break;
        case UP_LEFT_BACK:
            return DOWN_RIGHT_FRONT;
        break;
        case UP_RIGHT_FRONT:
            return DOWN_LEFT_BACK;
        break;
        case UP_RIGHT_BACK:
            return DOWN_LEFT_FRONT;
        break;
        case DOWN_LEFT:
            return UP_RIGHT;
        break;
        case DOWN_RIGHT:
            return UP_LEFT;
        break;
        case DOWN_FRONT:
            return UP_BACK;
        break;
        case DOWN_BACK:
            return UP_FRONT;
        break;
        case DOWN_LEFT_FRONT:
            return UP_RIGHT_BACK;
        break;
        case DOWN_LEFT_BACK:
            return UP_RIGHT_FRONT;
        break;
        case DOWN_RIGHT_FRONT:
            return UP_LEFT_BACK;
        break;
        case DOWN_RIGHT_BACK:
            return UP_LEFT_FRONT;
        break;
        default:
            return INVALID_DIR;
        break;
    }
}

Vector3f EnergyGrid::getDirectionVectorF(char direction)
{
    Vector3i intVector = getDirectionVector(direction);
    Vector3f floatVector(intVector.x, intVector.y, intVector.z);
    floatVector.normalize();
    return floatVector;
}

Vector3i EnergyGrid::getDirectionVector(char direction)
{
    return mDirectionVectors[direction];
}

bool EnergyGrid::pullEnergy(unsigned int nodeIndex, float energy, char direction)
{

    TransferNode& node = mTransferGraph[nodeIndex];
    Vector3i& coord = node.mVoxelCoord;
    VoxelData& voxel = mVoxelData[coord.x][coord.y][coord.z];

    //std::cout << "Pulling Energy from (" << coord.x << ", " << coord.y << ", " << coord.z << ")" << std::endl;

    if(//voxel.mDestroyed ||
       voxel.mSnapped)
       {
           return false; //No longer valid for transfer.
       }

    if(!node.mSource)
    {
        //Stress and pressure due to pulling the energy.
        if(direction != INVALID_DIR) //this is not the first voxel in the proccess.
        {
            float pressureFactor = mCurrentMap->at(direction);
            float stressFactor = 1.0f - pressureFactor;
            pressureVoxel(coord, energy * pressureFactor);
            stressVoxel(coord, energy * stressFactor);
        }

        //Pull energy from feeder


        if(node.mFeeder < 0)
        {
            //std::cout << "Seperated!" << std::endl;
            //Has been seperated, so repair feeders.

            std::vector<unsigned int> potentialFeeders;
            std::vector<char> potentialFeederDirections;
            std::vector<unsigned int> potentialFeederGeneration;

            int lowestGeneration = -1;
            float highestStress = 0.0f;
            float previousStress = 1.0f - fabs(mCurrentMap->at(node.mFeederDirection));

            for(unsigned int i = DIRECTION_START; i < INVALID_DIR; i++)
            {
                Vector3i adjanctCoord = coord + getDirectionVector(i);
                int nodeIndex = mTransferMap[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];
                VoxelData& adjanctVoxel = mVoxelData[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];

                //Node has already been added to graph
                if(nodeIndex >= 0)
                {
                    TransferNode& adjanctNode = mTransferGraph[nodeIndex];

                    //Check that the adjanct node is not fed by this one.
                    bool valid = true;
                    char adjanctToMe = getReverseDirection(i);

                    if(adjanctNode.mFeederDirection == adjanctToMe || adjanctNode.mDeadEnd) valid = false;

                    if(!(//adjanctVoxel.mDestroyed ||
                         adjanctVoxel.mSnapped) && valid)
                    {
                        potentialFeeders.push_back(nodeIndex);
                        potentialFeederDirections.push_back(i);
                        potentialFeederGeneration.push_back(adjanctNode.mGeneration);
                        if(potentialFeederGeneration.back() < lowestGeneration || lowestGeneration == -1) lowestGeneration = potentialFeederGeneration.back();
                    }
                }
            }



            for(unsigned int i = 0; i < potentialFeeders.size(); i++)
            {
                if(potentialFeederGeneration[i] == lowestGeneration)
                {
                     float potentialFeederStress = 1.0f - fabs(mCurrentMap->at(potentialFeederDirections[i]));
                    if(potentialFeederStress > highestStress || highestStress == 0.0f)
                    {
                        highestStress = potentialFeederStress;
                        node.mFeeder = potentialFeeders[i];
                        node.mFeederDirection = potentialFeederDirections[i];
                    }
                }
            }

            if(node.mFeeder = -1)
            {
                node.mDeadEnd = true;
                return false; //Could not get any more feeders, so this node is a dead end.
            }
        }

        if(node.mFeeder >= 0)
        {
            char transferDirection = getReverseDirection(node.mFeederDirection);

            //Apply stress and pressure from feeder
            float pressureFactor = mCurrentMap->at(transferDirection);
            float stressFactor = 1.0f - pressureFactor;

            pressureVoxel(coord, energy * pressureFactor);
            stressVoxel(coord, energy * stressFactor);

            //std::cout << "Pulling Energy from neighbour in direction: " << (int)transferDirection << std::endl;

            if(!pullEnergy(node.mFeeder, energy, transferDirection))
            {
                //std::cout << "Neighbour not valid." << std::endl;
                //Node not valid any more so remove from feeders.
                node.mFeeder = -1;
            }
            else
            {
                //std::cout << "Got Energy" << std::endl;
            }
        }


    }
    else
    {
        //std::cout << "Is Source" << std::endl;
    }
    if(//voxel.mDestroyed ||
       voxel.mSnapped)
    {
        return false; //No longer valid for transfer.
    }
    else return true;
}

void EnergyGrid::pressureVoxel(Vector3i voxelCoord, float pressure)
{
    if(validCoord(voxelCoord) && pressure > 0)
    {
        VoxelData& voxel = mVoxelData[voxelCoord.x][voxelCoord.y][voxelCoord.z];
        voxel.mPressure += pressure;

        float pressureLimit = getMatter()->getMaterial()->getPressureLimit()*voxel.mStrength;
        if(voxel.mPressure > pressureLimit)
        {
            voxel.mDestroyed = true;
            mDestructionOccured = true;
        }

    }
}

void EnergyGrid::stressVoxel(Vector3i voxelCoord, float stress)
{
    if(validCoord(voxelCoord) && stress > 0)
    {
        VoxelData& voxel = mVoxelData[voxelCoord.x][voxelCoord.y][voxelCoord.z];
        voxel.mStress += stress;

        float stressLimit = getMatter()->getMaterial()->getStressLimit()*voxel.mStrength;
        if(voxel.mStress > stressLimit)
        {
            voxel.mSnapped = true;
            mSnappingOccured = true;
        }

    }
}

/*
    Divide up the voxel field into several new shapes seperated by the broken and snapped voxels.
    Snapped voxels are added to the shape, but cannot expand any further.
    Broken voxels are not added to the shape.
*/
bool EnergyGrid::separate(std::vector<VoxelField>& voxelFieldArray)
{
    try
    {


    if(!mDestructionOccured && !mSnappingOccured)
    return false;

    //std::cout << "Starting Breakage: ";

    //Similar to building the transfer graph, we need to have a map and a vector
    bool noShapesLeft = false;
    unsigned int iterations = 0;

    unsigned int shapeNumber = 1;
    unsigned int shapeCount = 0;

    unsigned int totalVoxels = getMatter()->getVoxelField()->getNumVoxels();


    //Similar to building the transfer graph, we need to have a map.
    //The map stores a 32 bit int, each bit is a boolean that represents owner ship of a shape.
    std::vector<std::vector<std::vector<unsigned int>>> map(32, std::vector<std::vector<unsigned int>>(32, std::vector<unsigned int>(32, 0)));
    std::vector<std::vector<Vector3i>> voxelsInShapes;
    std::vector<float> shapesTotalEnergyR;
    std::vector<float> shapesTotalEnergyP;

    int x = 0;
    int y = 0;
    int z = 0;

    while(!noShapesLeft)
    {
        voxelsInShapes.push_back(std::vector<Vector3i>());


        float totalEnergyRecieving = 0.0f; //Total energy in recieveing direction.
        float totalEnergyProjecting = 0.0f; //Total energy in projecting direction.

        const VoxelData* masterVoxel;

        //Select first voxel for shape.
        bool foundValid = false;
        for(x = 0; x < 32 && !foundValid; x++)
        {
            for(y = 0; y < 32 && !foundValid; y++)
            {
                for(z = 0; z < 32 && !foundValid; z++)
                {
                    int voxelShape = map[x][y][z];
                    if(voxelShape == 0) //Must not be a member of another shape.
                    {
                        VoxelData& voxel = mVoxelData[x][y][z];
                        if(voxel.mFull)
                        {
                            voxelsInShapes[shapeCount].push_back(Vector3i(x,y,z));
                            masterVoxel = &mVoxelData[x][y][z];
                            totalEnergyProjecting += voxel.mEnergyInProjectedDirection;
                            totalEnergyRecieving += voxel.mEnergyInRecievingDirection;
                            map[x][y][z] = shapeNumber;
                            foundValid = true;
                        }
                    }
                }
            }
        }


        unsigned int oldSize = 0;
        unsigned int newSize = voxelsInShapes[shapeCount].size();
        unsigned int startOfStep = 0;
        unsigned int endOfStep = newSize;

        float currentShapePressure = masterVoxel->mPressure;
        float currentShapeStress = masterVoxel->mStress;

        bool limitBreak = false;

        while(oldSize != newSize && !limitBreak)
        {
            oldSize = newSize;
            endOfStep = newSize;

            for(unsigned int i = startOfStep; i < endOfStep; i++)
            {
                Vector3i currentCoord = voxelsInShapes[shapeCount][i];
                const VoxelData& currentVoxel = mVoxelData[currentCoord.x][currentCoord.y][currentCoord.z];

                if(!(!masterVoxel->mSnapped && currentVoxel.mSnapped))
                {
                    for(unsigned int k = DIRECTION_START; k < LEFT_FRONT; k++)
                    {
                        Vector3i adjanctCoord = currentCoord + getDirectionVector(k);
                        if(validCoord(adjanctCoord))
                        {
                            unsigned int voxelShape = map[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];

                            //Check not already in shape.
                            if(!(voxelShape & shapeNumber))
                            {
                                //Check is valid
                                VoxelData& adjanctVoxel = mVoxelData[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];
                                if(adjanctVoxel.mFull)
                                {
                                    bool canExpand = false;
                                    if(masterVoxel->mDestroyed && adjanctVoxel.mDestroyed)
                                    {
                                        float grouping = getMatter()->getMaterial()->getPressureGrouping();
                                        canExpand = fabs(currentVoxel.mPressure - adjanctVoxel.mPressure) < grouping;
                                    }
                                    else if(masterVoxel->mSnapped && adjanctVoxel.mSnapped)
                                    {
                                        float grouping = getMatter()->getMaterial()->getStressGrouping();
                                        canExpand = fabs(currentVoxel.mStress - adjanctVoxel.mStress) < grouping;
                                    }
                                    else if(!(masterVoxel->mSnapped || masterVoxel->mDestroyed))
                                    {
                                        canExpand = !adjanctVoxel.mDestroyed;
                                    }

                                    if(canExpand)
                                    {
                                        if(masterVoxel->mDestroyed) currentShapePressure += adjanctVoxel.mPressure;
                                        if(masterVoxel->mSnapped) currentShapeStress += adjanctVoxel.mStress;

                                        voxelsInShapes[shapeCount].push_back(adjanctCoord);

                                        totalEnergyProjecting += adjanctVoxel.mEnergyInProjectedDirection;
                                        totalEnergyRecieving += adjanctVoxel.mEnergyInRecievingDirection;

                                        //Add this shapes bit to the map.
                                        map[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z] = voxelShape | shapeNumber;
                                    }

                                }
                            }
                        }
                    }
                }
            }

            if(currentShapePressure > getMatter()->getMaterial()->getDebrisPressureLimit()
               || currentShapeStress > getMatter()->getMaterial()->getDebrisStressLimit())
            {
                limitBreak = true;
            }

            startOfStep = endOfStep;
            newSize = voxelsInShapes[shapeCount].size();
            //std::cout << newSize << std::endl;
        }

        shapesTotalEnergyP.push_back(totalEnergyProjecting);
        shapesTotalEnergyR.push_back(totalEnergyRecieving);

        //Shift 1 bit to left.
        shapeNumber << 1;

        if(voxelsInShapes[shapeCount].size() < 1)
        {
            noShapesLeft = true;
        }
        else
        {
            shapeCount++;
        }

    }

    //Shapes have been created, time to create Matter for each shape.
    if(shapeCount < 2 && voxelsInShapes[0].size() == totalVoxels) return false;

    //TODO: Actually do that, for now just return true.
    else
    {
        voxelFieldArray.clear();
        for(unsigned int i = 0; i < shapeCount; i++)
        {
            voxelFieldArray.push_back(VoxelField());
            voxelFieldArray.back().buildStart();
            for(unsigned int k = 0; k < voxelsInShapes[i].size(); k++)
            {
                voxelFieldArray[i].buildSetVoxel(voxelsInShapes[i][k], 1);
            }
            voxelFieldArray.back().buildEnd();
        }
        return true;
    }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }
}

bool EnergyGrid::validCoord(Vector3i target)
{
    //Possibly replace with non-hard coded sizes one day.
    bool valid = (target.x >= 0 && target.x <= 31 &&
       target.y >= 0 && target.y <= 31 &&
       target.z >= 0 && target.z <= 31);

    if(!valid)
    {
        int i = 0;
    }
    return valid;
}

Matter* EnergyGrid::getMatter()
{
    return mMatterNode->getMatter();
}

MatterNode* EnergyGrid::getMatterNode()
{
    return mMatterNode;
}

void EnergyGrid::setEnergyPerVoxel(float energyPerVoxel)
{
    mEnergyPerVoxel = energyPerVoxel;
}

void EnergyGrid::addBridge(Vector3i localVoxel, Vector3i partnerVoxel)
{
    int key = getBridgeKey(localVoxel);
    int point = getBridgeKey(partnerVoxel);
    mBridges.insert(std::pair<int, int>(key, point));
}

void EnergyGrid::updateRenderData()
{

    for(unsigned int x = 0; x < 32; x++)
    {
        for(unsigned int y = 0; y < 32; y++)
        {
            for(unsigned int z = 0; z < 32; z++)
            {
                VoxelData& voxel = mVoxelData[x][y][z];
                if(voxel.mFull)
                {
                    getMatter()->addPressureVertex(voxel.mPressure, voxel.mStress, voxel.mStrength, Vector3f(x, y, z));
                }
            }
        }
    }
    for(auto i = mBridges.begin(); i != mBridges.end(); i++)
    {
        Vector3i localInMe = getBridgeVector(i->first);
        getMatter()->addEnergyBridge(localInMe, true);
    }

}

//Constant Direction Vectors:

const Vector3i EnergyGrid::mDirectionVectors[27] = {
    Vector3i(0,0,-1), //DOWN
    Vector3i(0,0,1), //UP
    Vector3i(-1,0,0), //LEFT
    Vector3i(1,0,0), //RIGHT
    Vector3i(0,1,0), //FRONT
    Vector3i(0,-1,0), //BACK
    Vector3i(-1,1,0), //LEFT_FRONT
    Vector3i(-1,-1,0), //LEFT_BACK
    Vector3i(1,1,0), //RIGHT_FRONT
    Vector3i(1,-1,0), //RIGHT_BACK
    Vector3i(-1,0,1), //UP_LEFT
    Vector3i(1,0,1), //UP_RIGHT
    Vector3i(0,1,1), //UP_FRONT
    Vector3i(0,-1,1), //UP_BACK
    Vector3i(-1,1,1), //UP_LEFT_FRONT
    Vector3i(-1,-1,1), //UP_LEFT_BACK
    Vector3i(1,1,1), //UP_RIGHT_FRONT
    Vector3i(1,-1,1), //UP_RIGHT_BACK
    Vector3i(-1,0,-1), //DOWN_LEFT
    Vector3i(1,0,-1), //DOWN_RIGHT
    Vector3i(0,1,-1), //OWN_FRONT
    Vector3i(0,-1,-1), //DOWN_BACK
    Vector3i(-1,1,-1), //DOWN_LEFT_FRONT
    Vector3i(-1,-1,-1), //DOWN_LEFT_BACK
    Vector3i(1,1,-1), //DOWN_RIGHT_FRONT
    Vector3i(1,-1,-1), //DOWN_RIGHT_BACK
    Vector3i(0,0,0) //INVALID_DIR
};

const Vector3f& EnergyGrid::getExternalTransferDelta()
{
    if(mFirstExternalTransfer)
    {

        //Quick bit of setup
        unsigned int x = 0;
        unsigned int y = 1;
        unsigned int z = 2;

        //Find the direction our partners energy is travelling in.
        Vector3f partnerDirection = mCollisionPartner->getEnergyVector();
        const Matrix4D& worldToLocal = mCollisionPartner->getMatter()->getRigidBody()->getInvertedTransform();
        partnerDirection = worldToLocal.rotateVector(partnerDirection);
        partnerDirection.normalize();

        //The energy vector starts at pointCoord and travels along partnerDirection
        //P = pointCoord + t * partnerDirection where P is the point on the line and t is some number

        //The most significant direction of the vector will give us the idea as to which plane to divide the energy along.
        unsigned int sig;
        int direction;
        unsigned int up;
        unsigned int left;

        if(fabs(partnerDirection.x) >= fabs(partnerDirection.y) && fabs(partnerDirection.x) >= fabs(partnerDirection.z))
        {
            //Most significant direction is X
            if(partnerDirection.x > 0)
            {
                sig = x;
                direction = 1;
            }
            else
            {
                sig = x;
                direction  = -1;
            }
            up = y;
            left = z;
        }
        else if(fabs(partnerDirection.y) >= fabs(partnerDirection.x) && fabs(partnerDirection.y) >= fabs(partnerDirection.z))
        {
            //...is Y
            if(partnerDirection.y > 0)
            {
                sig = y;
                direction = 1;
            }
            else
            {
                sig = y;
                direction  = -1;
            }
            up = x;
            left = z;
        }
        else
        {
            //...is Z
            if(partnerDirection.z > 0)
            {
                sig = z;
                direction = 1;
            }
            else
            {
                sig = z;
                direction  = -1;
            }
            up = y;
            left = x;
        }

        float ratio = 1.0f/fabs(partnerDirection.get(sig));
        mExternalTransferDelta.set(sig, ratio*partnerDirection.get(sig));
        mExternalTransferDelta.set(left, ratio*partnerDirection.get(left));
        mExternalTransferDelta.set(up, ratio*partnerDirection.get(up));

        mFirstExternalTransfer = false;
        mExternalTransferSig = sig;
        mExternalTransferLeft = left;
        mExternalTransferUp = up;
    }

    return mExternalTransferDelta;

}

const Vector3f& EnergyGrid::getInternalTransferDelta()
{
    if(mFirstInternalTransfer)
    {

        //Quick bit of setup
        unsigned int x = 0;
        unsigned int y = 1;
        unsigned int z = 2;

        //Find the direction our partners energy is travelling in.
        Vector3f myDirection = -getEnergyVector();
        const Matrix4D& worldToLocal = getMatter()->getRigidBody()->getInvertedTransform();
        myDirection = worldToLocal.rotateVector(myDirection);
        myDirection.normalize();

        //The energy vector starts at pointCoord and travels along partnerDirection
        //P = pointCoord + t * partnerDirection where P is the point on the line and t is some number

        //The most significant direction of the vector will give us the idea as to which plane to divide the energy along.
        unsigned int sig;
        int direction;
        unsigned int up;
        unsigned int left;

        if(fabs(myDirection.x) >= fabs(myDirection.y) && fabs(myDirection.x) >= fabs(myDirection.z))
        {
            //Most significant direction is X
            if(myDirection.x > 0)
            {
                sig = x;
                direction = 1;
            }
            else
            {
                sig = x;
                direction  = -1;
            }
            up = y;
            left = z;
        }
        else if(fabs(myDirection.y) >= fabs(myDirection.x) && fabs(myDirection.y) >= fabs(myDirection.z))
        {
            //...is Y
            if(myDirection.y > 0)
            {
                sig = y;
                direction = 1;
            }
            else
            {
                sig = y;
                direction  = -1;
            }
            up = x;
            left = z;
        }
        else
        {
            //...is Z
            if(myDirection.z > 0)
            {
                sig = z;
                direction = 1;
            }
            else
            {
                sig = z;
                direction  = -1;
            }
            up = y;
            left = x;
        }

        float ratio = 1.0f/fabs(myDirection.get(sig));
        mInternalTransferDelta.set(sig, ratio*myDirection.get(sig));
        mInternalTransferDelta.set(left, ratio*myDirection.get(left));
        mInternalTransferDelta.set(up, ratio*myDirection.get(up));

        mFirstExternalTransfer = false;
        mInternalTransferSig = sig;
        mInternalTransferLeft = left;
        mInternalTransferUp = up;
    }

    return mInternalTransferDelta;
}

