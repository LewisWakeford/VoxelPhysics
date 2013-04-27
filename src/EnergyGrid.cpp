#include "EnergyGrid.h"

#include "Matter.h"
#include "MatterNode.h"

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
                    mVoxelData[x][y][z].mStrength = 1.0f;
                }
            }
        }
    }
    mDestructionOccured = false;
    mSnappingOccured = false;
}

EnergyGrid::~EnergyGrid()
{
    //dtor
}

void EnergyGrid::setEnergy(const Vector3f& energy)
{
    mEnergyVector = energy;
    mEnergyVectorLocal = getMatter()->getRigidBody()->getInvertedTransform().rotateVector(energy);
}

Vector3f EnergyGrid::getEnergyVector()
{
    return mEnergyVector;
}

void EnergyGrid::setCollisionPartner(EnergyGrid* energyGrid)
{
    mCollisionPartner = energyGrid;
}

void EnergyGrid::buildMaps(const Vector3f& otherEnergy)
{
    //Get the direction of the energy.
    Vector3f myDirection = mEnergyVectorLocal.normalized();
    Vector3f otherDirection = otherEnergy.normalized();

    //Transform the other direction into voxel space.
    otherDirection = getMatter()->getRigidBody()->getTransform().inverted().rotateVector(otherDirection);

    //Build Maps

    //Projector.
    float totalPressure = 0.0f;
    for(unsigned int i = 0; i < 26; i++)
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
}

void EnergyGrid::setAsProjector()
{
    mCurrentMap = mProjectorMap;
    mCurrentPressureDirections = &mPressureDirectionsP;
    mIsReciever = false;
}

void EnergyGrid::setAsReciever()
{
    mCurrentMap = mRecieverMap;
    mCurrentPressureDirections = &mPressureDirectionsR;
    mIsReciever = true;
}

void EnergyGrid::setInitialEnergy(float energy)
{
    unsigned int totalVoxels = getMatter()->getVoxelField()->getNumVoxels();
    float energyPerVoxel = energy / totalVoxels;

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
    //MANDATORY OPTIMISATION:
    //Start at back of shape and work forward, as this may result in energy transfer in a single pass.

    int zStart = 0;
    int zEnd = 31;
    int zDirection = 1;

    int yStart = 0;
    int yEnd = 31;
    int yDirection = 1;

    int xStart = 0;
    int xEnd = 31;
    int xDirection = 1;

    if(mEnergyVectorLocal.x < 0)
    {
        xStart = 31;
        xEnd = 0;
        xDirection = -1;
    }
    if(mEnergyVectorLocal.y < 0)
    {
        yStart = 31;
        yEnd = 0;
        yDirection = -1;
    }
    if(mEnergyVectorLocal.z < 0)
    {
        zStart = 31;
        zEnd = 0;
        zDirection = -1;
    }

    //Create two lists of voxels, swap between them each pass.
    std::vector<Vector3i> voxelListA;
    std::vector<Vector3i> voxelListB;
    std::vector<Vector3i>* currentList = &voxelListA;
    std::vector<Vector3i>* otherList = &voxelListB;
    bool listIsA = true;

    std::cout << "    Building Initial List..." << std::endl;
    for(int z = zStart; z != zEnd; z += zDirection)
    {
        for(int y = yStart; y != yEnd; y += yDirection)
        {
            for(int x = xStart; x != xEnd; x += xDirection)
            {
                if(mVoxelData[x][y][z].mFull)
                {
                    //Already done else where
                    //mVoxelData[x][y][z].addEnergy(mIsReciever, startingEnergyPerVoxel);
                    currentList->push_back(Vector3i(x, y, z));
                }
            }
        }
    }


    while(currentList->size() > 0)
    {
        std::cout << "    Transfer Step..." << std::endl;
        //Transfer energy from all voxels.
        for(unsigned int i = 0; i < currentList->size(); i++)
        {
            directTransferVoxel(currentList->at(i));
        }
        std::cout << "    Rebuilding List..." << std::endl;
        //Check if energy can be transferred again.
        for(unsigned int i = 0; i < currentList->size(); i++)
        {
            Vector3i coord = currentList->at(i);
            VoxelData& voxel = mVoxelData[coord.x][coord.y][coord.z];
            if(voxel.getEnergy(mIsReciever) > mEnergyPerVoxel)
            {
                otherList->push_back(coord);
            }
        }
        //Swap lists
        std::vector<Vector3i>* temp = currentList;
        currentList = otherList;
        otherList = temp;
        otherList->clear();
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
    buildTransferGraph();
    for(unsigned int i = 0; i < mTransferGraph.size(); i++)
    {
        TransferNode& currentNode = mTransferGraph[i];
        if(!currentNode.mSource)
        {
            Vector3i coord = currentNode.mVoxelCoord;
            VoxelData& voxel = mVoxelData[coord.x][coord.y][coord.z];

            float requiredEnergy = mEnergyPerVoxel - voxel.getEnergy(mIsReciever);
            if(requiredEnergy > 0) pullEnergy(i, requiredEnergy, 30);
            voxel.setEnergy(mIsReciever, mEnergyPerVoxel);
        }
    }
}

void EnergyGrid::buildTransferGraph()
{
    mTransferGraph.clear();

    unsigned int startOfStep = 0;
    unsigned int endOfStep = 0;

    //Reset the transfermap
    mTransferMap = std::vector<std::vector<std::vector<int>>>(32, std::vector<std::vector<int>>(32, std::vector<int>(32, -1)));

    //Add source voxels to graph
    for(unsigned int x = 0; x < 32; x++)
    {
        for(unsigned int y = 0; y < 32; y++)
        {
            for(unsigned int z = 0; z < 32; z++)
            {
                VoxelData& voxel = mVoxelData[x][y][z];
                if(voxel.mFull && !voxel.mDestroyed && voxel.getEnergy(mIsReciever) >= mEnergyPerVoxel)
                {
                    mTransferGraph.push_back(TransferNode(true, Vector3i(x, y, z)));
                    mTransferGraph.back().mGeneration = 0;
                    mTransferMap[x][y][z] = mTransferGraph.size()-1;
                    endOfStep++;
                }
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
            TransferNode currentNode = mTransferGraph[i];
            const Vector3i coord = currentNode.mVoxelCoord;

            //Get coords of adjanct nodes
            for(unsigned int k = 0; k < 26; k++)
            {
                Vector3i adjanctCoord = coord + getDirectionVector(k);
                if(validCoord(adjanctCoord))
                {
                    int nodeIndex = mTransferMap[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];

                    //Node has already been added to graph, but is "younger" than this node.
                    if(nodeIndex >= 0 && nodeIndex > endOfStep)
                    {
                        TransferNode& adjanctNode = mTransferGraph[nodeIndex];
                        char previousFeederDirection = getReverseDirection(adjanctNode.mFeederDirection);
                        float previousFeederStress = 1.0f - fabs(mCurrentMap[previousFeederDirection]);
                        float stress = 1.0f - fabs(mCurrentMap[k]);

                        //If this voxel is a better feeder, then use it instead of the previous one.
                        if(stress > previousFeederStress)
                        {
                            adjanctNode.mFeeder = i;
                            adjanctNode.mFeederDirection = getReverseDirection(k);
                        }
                    }
                    else if(nodeIndex < 0)
                    {
                        //Node has not been added, must check it is full.
                        VoxelData& voxel = mVoxelData[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z];
                        if(voxel.mFull && !(voxel.mDestroyed || voxel.mSnapped))
                        {
                            mTransferGraph.push_back(TransferNode(false, adjanctCoord));
                            mTransferMap[adjanctCoord.x][adjanctCoord.y][adjanctCoord.z] = mTransferGraph.size()-1;
                            TransferNode& newNode = mTransferGraph.back();
                            newNode.mFeeder = i;
                            newNode.mFeederDirection = getReverseDirection(k);
                            newNode.mGeneration = generation;
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

void EnergyGrid::directTransferVoxel(Vector3i sourceVoxel)
{
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
        }
        //Only take some
        else if(energyToTransfer >= 0)
        {
            pressureVoxel(sourceVoxel, energy);
            source.setEnergy(mIsReciever, energyToTransfer-energy+mEnergyPerVoxel);
        }

        if(energyToTransfer >= 0)
        {

            pressureVoxel(sourceVoxel, energyToTransfer);
            source.setEnergy(mIsReciever, mEnergyPerVoxel);
            return energyToTransfer;
        }
    }
    return 0.0f;
}

bool EnergyGrid::getBridge(int key, Vector3f& rBridgePoint)
{
    std::unordered_map<int, Vector3f>::const_iterator it = mBridges.find(key);
    if(it == mBridges.end() )
    {
        return false;
    }
    else
    {
        rBridgePoint = it->second;
        return true;
    }
}

void EnergyGrid::transferEnergyTo(Vector3i targetVoxel, char direction, float energy)
{
    int bridgeKey = getBridgeKey(targetVoxel);
    Vector3f bridgePoint;
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

//Returns the key to the bridge.
//Which is the x-y-z coords compressed into an int.
int EnergyGrid::getBridgeKey(Vector3i voxelCoord)
{
    //Max value, 33 so 6 bit ints.
    return voxelCoord.x << 12 | voxelCoord.y << 6 | voxelCoord.z;
}

void EnergyGrid::transferExternalEnergyTo(Vector3f pointCoord, float energy)
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

    //Find where the energy vector intersects each of the 32 planes in the significant direction.
    //At each intersection find the four closest voxels in that plane to the intersection point.
    //Spread energy between them based on distance from point of intersection.

    float energyPool = energy;

    Vector3i* previousVoxels = 0;
    float* previousEnergy = 0;

    unsigned int start = 0;
    unsigned int end = 32;

    if(direction < 0)
    {
        start = 32;
        end = 0;
    }

    for(unsigned int i = start; i != end; i += direction)
    {
        //NOTE: We can assume that the line is not parrallel to the plane as we change the plane we intersect with based on direction of energy.
        Vector3f pointOfIntersection;

        //Equation of plane is sig = pointInPlane.sig

        //Equation of line is:
        // sig = pointCoord.sig + (partnerDirection.sig * t)
        // up = pointCoord.up + (partnerDirection.up * t)
        // left = pointCoord.left + (partnerDirection.left * t)

        //Recreate plane with t and solve for t
        // pointCoord.sig + (partnerDirection.sig * t) = pointInPlane.sig
        // partnerDirection.sig * t = pointInPlane.sig - pointCoord.sig
        // t = (pointInPlane.sig - pointCoord.sig) / partnerDirection.sig


        float t = (float(i) - pointCoord.get(sig)) / partnerDirection.get(sig);

        pointOfIntersection.set(sig, float(i));
        pointOfIntersection.set(up, pointCoord.get(up) + (partnerDirection.get(up) * t));
        pointOfIntersection.set(left, pointCoord.get(left) + (partnerDirection.get(left) * t));

        Vector3i closestVoxels[4];
        float voxelWeights[4];
        bool voxelValid[4];
        int validVoxels = 4;

        int upCoord = ceilf(pointOfIntersection.get(up));
        int downCoord = floorf(pointOfIntersection.get(up));
        int leftCoord = ceilf(pointOfIntersection.get(left));
        int rightCoord = floorf(pointOfIntersection.get(left));

        if(sig == x)
        {
            closestVoxels[0].set(i, upCoord, leftCoord);
            closestVoxels[1].set(i, downCoord, leftCoord);
            closestVoxels[2].set(i, upCoord, rightCoord);
            closestVoxels[3].set(i, downCoord, rightCoord);
        }
        else if(sig == y)
        {
            closestVoxels[0].set(upCoord, i, leftCoord);
            closestVoxels[1].set(downCoord, i, leftCoord);
            closestVoxels[2].set(upCoord, i, rightCoord);
            closestVoxels[3].set(downCoord, i, rightCoord);
        }
        else
        {
            closestVoxels[0].set(leftCoord, upCoord, i);
            closestVoxels[1].set(leftCoord, downCoord, i);
            closestVoxels[2].set(rightCoord, upCoord, i);
            closestVoxels[3].set(rightCoord, downCoord, i);
        }

        for(int k = 0; k < 4; k++)
        {
            float verticalDistance = closestVoxels[k].get(up) - pointOfIntersection.get(up);
            float horizontalDistance = closestVoxels[k].get(left) - pointOfIntersection.get(left);

            float weight = fabs(verticalDistance) * fabs(horizontalDistance);
            voxelWeights[k] = weight;
        }

        //Check for validity
        for(int k = 0; k < 4; k++)
        {
            voxelValid[k] = validCoord(closestVoxels[k]);

            if(!voxelValid[k])
            {
                validVoxels -= 1;
                //If invalid, redistribute weight
                if(validVoxels > 0)
                {

                    float shareRatio = 1.0f / validVoxels;
                    for(int j = 0; j < 4; j++)
                    {
                        if(!voxelValid[j])
                        {
                            voxelWeights[j] += voxelWeights[k] * shareRatio;
                        }

                    }
            }

            voxelWeights[k] = 0.0f;
            }

        }

        if(previousVoxels && previousEnergy)
        {
            energyPool += transferEnergyFrom(previousVoxels[0], previousEnergy[0]);
            energyPool += transferEnergyFrom(previousVoxels[1], previousEnergy[1]);
            energyPool += transferEnergyFrom(previousVoxels[2], previousEnergy[2]);
            energyPool += transferEnergyFrom(previousVoxels[3], previousEnergy[3]);
            delete[] previousVoxels;
            delete[] previousEnergy;
        }

        previousVoxels = new Vector3i[4];
        previousEnergy = new float[4];
        previousVoxels[0] = closestVoxels[0];
        previousVoxels[1] = closestVoxels[1];
        previousVoxels[2] = closestVoxels[2];
        previousVoxels[3] = closestVoxels[3];

        for(int k = 0; k < 4; k++)
        {
            if(voxelValid[k])
            {
                float energyToTransfer = energyPool*voxelWeights[k];
                directTransferTo(closestVoxels[k], energyToTransfer);
                previousEnergy[k] = energyToTransfer;
                energyPool -= energyToTransfer;
            }
            else
            {
                previousEnergy[k] = 0.0f;
            }
        }

        if(i == end-1)
        {
            delete[] previousVoxels;
            delete[] previousEnergy;
        }
    }

}

char EnergyGrid::getReverseDirection(char direction)
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
            return 0xFF;
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

    if(voxel.mDestroyed || voxel.mSnapped) return false; //No longer valid for transfer.

    if(!node.mSource)
    {

        VoxelData& voxel = mVoxelData[coord.x][coord.y][coord.z];

        //Stress and pressure due to pulling the energy.
        if(direction < 26) //if above 26 this is the first voxel in the proccess.
        {
            float pressureFactor = mCurrentMap[direction];
            float stressFactor = 1.0f - pressureFactor;
            pressureVoxel(coord, energy * pressureFactor);
            stressVoxel(coord, energy * stressFactor);
        }

        //Pull energy from feeder
        if(node.mFeeder >= 0)
        {
            char transferDirection = getReverseDirection(node.mFeederDirection);

            //Apply stress and pressure from feeder
            float pressureFactor = mCurrentMap[transferDirection];
            float stressFactor = 1.0f - pressureFactor;

            pressureVoxel(coord, energy * stressFactor);
            stressVoxel(coord, energy * pressureFactor);

            if(!pullEnergy(node.mFeeder, energy, transferDirection))
            {
                //Node not valid any more so remove from feeders.
                node.mFeeder = -1;
            }
        }
        else
        {
            //Has been seperated, so repair feeders.

            std::vector<unsigned int> potentialFeeders;
            std::vector<char> potentialFeederDirections;
            std::vector<unsigned int> potentialFeederGeneration;

            int lowestGeneration = -1;
            float highestStress = 0.0f;
            float previousStress = 1.0f - fabs(mCurrentMap[node.mFeederDirection]);

            for(unsigned int i = 0; i < 26; i++)
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

                    if(!(adjanctVoxel.mDestroyed || adjanctVoxel.mSnapped) && valid)
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
                    float potentialFeederStress = 1.0f - fabs(mCurrentMap[potentialFeeders[i]]);
                    if(potentialFeederStress > highestStress || highestStress == 0.0f)
                    {
                        highestStress = potentialFeederStress;
                        node.mFeeder = potentialFeeders[i];
                        node.mFeederDirection = potentialFeederDirections[i];
                    }
                }
            }

        }
        if(node.mFeeder = -1)
        {
            node.mDeadEnd = true;
            return false; //Could not get any more feeders, so this node is a dead end.
        }
    }

    if(voxel.mDestroyed || voxel.mSnapped) return false; //No longer valid for transfer.
    else return true;
}

void EnergyGrid::pressureVoxel(Vector3i voxelCoord, float pressure)
{
    if(validCoord(voxelCoord))
    {
        VoxelData& voxel = mVoxelData[voxelCoord.x][voxelCoord.y][voxelCoord.z];
        voxel.mPressure += pressure;
        float pressureLimit = getMatter()->getMaterial()->getPressureLimit();
        if(voxel.mPressure > pressureLimit)
        {
            voxel.mDestroyed = true;
            mDestructionOccured = true;
        }
    }
}

void EnergyGrid::stressVoxel(Vector3i voxelCoord, float stress)
{
    if(validCoord(voxelCoord))
    {
        VoxelData& voxel = mVoxelData[voxelCoord.x][voxelCoord.y][voxelCoord.z];
        voxel.mStress += stress;
        float stressLimit = getMatter()->getMaterial()->getStressLimit();
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
    if(!mDestructionOccured && !mSnappingOccured)
    return false;

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

    while(!noShapesLeft)
    {
        voxelsInShapes.push_back(std::vector<Vector3i>());


        float totalEnergyRecieving = 0.0f; //Total energy in recieveing direction.
        float totalEnergyProjecting = 0.0f; //Total energy in projecting direction.

        //Select first voxel for shape.
        bool foundValid = false;
        for(unsigned int x = 0; x < 32 && !foundValid; x++)
        {
            for(unsigned int y = 0; y < 32 && !foundValid; y++)
            {
                for(unsigned int z = 0; z < 32 && !foundValid; z++)
                {
                    int voxelShape = map[x][y][z];
                    if(voxelShape == 0) //Must not be a member of another shape.
                    {
                        VoxelData& voxel = mVoxelData[x][y][z];
                        if(voxel.mFull && !voxel.mDestroyed && !voxel.mSnapped)
                        {
                            voxelsInShapes[shapeCount].push_back(Vector3i(x,y,z));
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

        while(oldSize != newSize)
        {
            oldSize = newSize;
            endOfStep = newSize;

            for(unsigned int i = startOfStep; i < endOfStep; i++)
            {
                const Vector3i& currentCoord = voxelsInShapes[shapeCount][i];
                const VoxelData& currentVoxel = mVoxelData[currentCoord.x][currentCoord.y][currentCoord.z];

                if(!currentVoxel.mSnapped)
                {
                    for(unsigned int k = 0; k < 26; k++)
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
                                if(adjanctVoxel.mFull && !adjanctVoxel.mDestroyed)
                                {
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

            startOfStep = endOfStep;
            newSize = voxelsInShapes[shapeCount].size();
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

void EnergyGrid::addBridge(Vector3i voxel, Vector3f point)
{
    int key = getBridgeKey(voxel);
    mBridges.insert(std::pair<int, Vector3f>(key, point));
}

void EnergyGrid::updateRenderData()
{
    getMatter()->clearPressureRendering();
    for(unsigned int x = 0; x < 32; x++)
    {
        for(unsigned int y = 0; y < 32; y++)
        {
            for(unsigned int z = 0; z < 32; z++)
            {
                VoxelData& voxel = mVoxelData[x][y][z];
                if(voxel.mFull)
                {
                    getMatter()->addPressureVertex(voxel.mPressure, voxel.mStress, Vector3f(x, y, z));
                }
            }
        }
    }
    getMatter()->setupPressureRendering();
}

//Constant Direction Vectors:

const Vector3i EnergyGrid::mDirectionVectors[26] = {
    Vector3i(0,0,1), //UP
    Vector3i(0,0,-1), //DOWN
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
    Vector3i(1,-1,-1) //DOWN_RIGHT_BACK
};
