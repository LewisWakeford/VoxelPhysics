#include "DestructionEngine.h"
#include "Matter.h"
#include "MatterNode.h"

#include "App.h"
#include "SceneGraph.h"

DestructionEngine::DestructionEngine(App* app)
{
    mApp = app;
}

DestructionEngine::~DestructionEngine()
{
    //dtor
}

void DestructionEngine::processSet(const MatterCollisionSet& set)
{
    for(int i = 0; i < set.numCollisions(); i++)
    {
        processCollision(set.get(i));
    }
}

unsigned int DestructionEngine::getGridIndex(MatterNode* matterNode)
{
    for(unsigned int i = 0; i < mEnergyGrids.size(); i++)
    {
        if(mEnergyGrids[i].getMatter()->getID() == matterNode->getMatter()->getID())
        {
            return i;
        }
    }
    mEnergyGrids.push_back(EnergyGrid(matterNode));
    return mEnergyGrids.size()-1;
}

void DestructionEngine::processCollision(const MatterCollision& collision)
{
    //Calculate the amounts of kinetic energy each Matter has.
    //Half em veee squared, but seperate velocity into direction and speed.

    const btVector3& firstVelocity = collision.getVelocityFirst();
    const btVector3& secondVelocity = collision.getVelocitySecond();

     float firstSpeed = firstVelocity.length();
    float secondSpeed = secondVelocity.length();

    Matter* first = collision.getFirst()->getMatter();
    Matter* second = collision.getSecond()->getMatter();

    btVector3 energyFirst = (0.5 * first->getMass()) * (firstVelocity.normalized() * (firstSpeed * firstSpeed));
    btVector3 energySecond = (0.5 * second->getMass()) * (firstVelocity.normalized() * (secondSpeed * secondSpeed));

    float energyFirstScalar = energyFirst.length();
    float energySecondScalar = energySecond.length();

    //Check that collision is not trivial:
    if(first->nonTrivial(energyFirstScalar + energySecondScalar) ||
        second->nonTrivial(energyFirstScalar + energySecondScalar))
    {
        std::cout << "\n\n****NON TRIVIAL***" << std::endl;
        unsigned int firstIndex = getGridIndex(collision.getFirst());
        unsigned int secondIndex = getGridIndex(collision.getSecond());

        Vector3f myEnergyFirst(energyFirst.x(), energyFirst.y(), energyFirst.z());
        Vector3f myEnergySecond(energySecond.x(), energySecond.y(), energySecond.z());

        mEnergyGrids[firstIndex].setEnergy(myEnergyFirst);
        mEnergyGrids[secondIndex].setEnergy(myEnergySecond);



        if(firstSpeed >= secondSpeed)
        {
            buildBridges(firstIndex, secondIndex, collision);
            transferEnergy(mEnergyGrids[firstIndex], mEnergyGrids[secondIndex]);
        }
        else
        {
            buildBridges(secondIndex, firstIndex, collision);
            transferEnergy(mEnergyGrids[secondIndex], mEnergyGrids[firstIndex]);
        }
    }
}

void DestructionEngine::transferEnergy(EnergyGrid& first, EnergyGrid& second)
{
    std::cout << "Starting Transfer..." << std::endl;
    first.setCollisionPartner(&second);
    second.setCollisionPartner(&first);

    const Vector3f& firstEnergy = first.getEnergyVector();
    const Vector3f& secondEnergy = second.getEnergyVector();

    std::cout << "Building Maps..." << std::endl;
    first.buildMaps(secondEnergy);
    second.buildMaps(firstEnergy);


    //Perform First's transfer into Second.

    //Calculate seconds energy in terms of first
    std::cout << "Performing setup..." << std::endl;
    Vector3f directionFirst = firstEnergy.normalized();
    float secondEnergyRelative = directionFirst.dot(secondEnergy);
    float firstEnergyScalar = firstEnergy.length();



    unsigned int totalVoxels = first.getMatter()->getVoxelField()->getNumVoxels() + second.getMatter()->getVoxelField()->getNumVoxels();
    float energyPerVoxel = (firstEnergyScalar + secondEnergyRelative) / totalVoxels;

    first.setEnergyPerVoxel(energyPerVoxel);
    second.setEnergyPerVoxel(energyPerVoxel);

    first.setAsProjector();
    second.setAsReciever();


    first.setInitialEnergy(firstEnergyScalar);
    second.setInitialEnergy(secondEnergyRelative);

    std::cout << "Performing direct transfer..." << std::endl;
    first.directTransfer();

    std::cout << "Performing indirect transfer..." << std::endl;
    first.indirectTransfer();

    std::cout << "Performing indirect transfer..." << std::endl;
    second.indirectTransfer();


    std::cout << "Performing setup..." << std::endl;
    //Do the same for second

    float remainingEnergyRatio = secondEnergyRelative / secondEnergy.length();
    Vector3f remainingEnergy = secondEnergy.mult(remainingEnergyRatio);
    float remainingEnergyScalar = remainingEnergy.length();

    //Vector3f directionSecond = remainingEnergy.normalized();
    //float firstEnergyRelative = directionSecond.dot(firstEnergy);

    energyPerVoxel = (remainingEnergyScalar) / totalVoxels;

    first.setEnergyPerVoxel(energyPerVoxel);
    second.setEnergyPerVoxel(energyPerVoxel);

    second.setAsProjector();
    first.setAsReciever();

    first.setInitialEnergy(0.0f);
    second.setInitialEnergy(remainingEnergyScalar);

    std::cout << "Performing direct transfer..." << std::endl;
    second.directTransfer();

    std::cout << "Performing indirect transfer..." << std::endl;
    second.indirectTransfer();

    std::cout << "Performing indirect transfer..." << std::endl;
    first.indirectTransfer();

    std::cout << "---SET---" << std::endl;
    std::cout << "First: " << firstEnergyScalar << std::endl;
    std::cout << "Second: " << secondEnergyRelative << std::endl;

    first.updateRenderData();
    second.updateRenderData();
}

void DestructionEngine::buildBridges(unsigned int firstIndex, unsigned int secondIndex, const MatterCollision& collision)
{
    std::cout << "Building Bridges..." <<std::endl;

    //Get collision bounding boxes for each matter.
    EnergyGrid& first = mEnergyGrids[firstIndex];
    EnergyGrid& second = mEnergyGrids[secondIndex];

    Matrix4D firstTransform = first.getMatter()->getRigidBody()->getTransform();
    Matrix4D firstTransformInverse = first.getMatter()->getRigidBody()->getInvertedTransform();
    BoundingBox firstBB = first.getMatter()->getVoxelField()->createTransformedBoundingBox(firstTransform, 1.0f);

    Matrix4D secondTransform = second.getMatter()->getRigidBody()->getTransform();
    Matrix4D secondTransformInverse = second.getMatter()->getRigidBody()->getInvertedTransform();
    BoundingBox secondBB = second.getMatter()->getVoxelField()->createTransformedBoundingBox(secondTransform, 1.0f);

    //Get the voxels closest to the collision point.
    //These are the only two guarenteed bridges.
    btVector3 collisionPointFirst = collision.getPoint().getPositionWorldOnA();
    Vector3i closestVoxelFirst = first.getMatter()->getVoxelField()->getVoxelClosestTo(firstTransformInverse, Vector3f(collisionPointFirst.x(), collisionPointFirst.y(), collisionPointFirst.z()));

    btVector3 collisionPointSecond = collision.getPoint().getPositionWorldOnB();
    Vector3i closestVoxelSecond = second.getMatter()->getVoxelField()->getVoxelClosestTo(secondTransformInverse, Vector3f(collisionPointSecond.x(), collisionPointSecond.y(), collisionPointSecond.z()));

    //Find potential bridges
    std::vector<Vector3i> potentialBridgesFirst;
    first.getMatter()->getVoxelField()->getPotentialBridges(firstTransformInverse, first.getEnergyVector().normalized(), potentialBridgesFirst);
    std::vector<Vector3i> potentialBridgesSecond;
    second.getMatter()->getVoxelField()->getPotentialBridges(secondTransformInverse, -(first.getEnergyVector()).normalized(), potentialBridgesSecond);

    std::vector<Vector3f> closeBridgesFirst;
    std::vector<unsigned int> closeBridgesFirstOriginal;
    std::vector<Vector3f> closeBridgesSecond;
    std::vector<unsigned int> closeBridgesSecondOriginal;


    Vector3f cOfMF = first.getMatter()->getVoxelField()->getCenterOfMass();

    for(int i = 0; i < potentialBridgesFirst.size(); i++)
    {
        //Transform each voxel into world space.
        Vector3f voxel = firstTransform.transformVertex(Vector3f(potentialBridgesFirst[i].x-cOfMF.x, potentialBridgesFirst[i].y-cOfMF.y, potentialBridgesFirst[i].z-cOfMF.z));

        //Test if voxel is within bounding box of the other matter.
        if(secondBB.pointInside(voxel))
        {
            closeBridgesFirst.push_back(voxel);
            closeBridgesFirstOriginal.push_back(i);
        }
    }

    Vector3f cOfMS = first.getMatter()->getVoxelField()->getCenterOfMass();
    for(int i = 0; i < potentialBridgesSecond.size(); i++)
    {
        //Transform each voxel into world space.
        Vector3f voxel = secondTransform.transformVertex(Vector3f(potentialBridgesSecond[i].x-cOfMS.x, potentialBridgesSecond[i].y-cOfMS.y, potentialBridgesSecond[i].z-cOfMS.z));

        //Test if voxel is within bounding box of the other matter.
        if(firstBB.pointInside(voxel))
        {
            closeBridgesSecond.push_back(voxel);
            closeBridgesSecondOriginal.push_back(i);
        }
    }

    float closeRadius = 2.0f;

    //Perform more advanced tests.
    for(int i = 0; i < closeBridgesFirst.size(); i++)
    {
        for(int k = 0; k < closeBridgesSecond.size(); k++)
        {
            if(closeBridgesFirst[i].distance(closeBridgesSecond[k]) < closeRadius)
            {
                Vector3f localPointFirst = firstTransformInverse.transformVertex(closeBridgesSecond[i]) + cOfMF;
                Vector3f localPointSecond  = secondTransformInverse.transformVertex(closeBridgesFirst[i]) + cOfMS;
                first.addBridge(potentialBridgesFirst[closeBridgesFirstOriginal[i]], localPointSecond);
                second.addBridge(potentialBridgesSecond[closeBridgesSecondOriginal[k]], localPointFirst);
            }
        }
    }
}

void DestructionEngine::checkForSeparation()
{
    for(unsigned int i = 0; i < mEnergyGrids.size(); i++)
    {
        MatterNode* original = mEnergyGrids[i].getMatterNode();
        std::vector<VoxelField> voxArray;
        if(mEnergyGrids[i].separate(voxArray))
        {
           //Create new matter nodes.
           std::cout << "BREAK!" << std::endl;

           for(unsigned int k = 0; k < voxArray.size(); k++)
           {
               SceneNodePtr matterNode(new MatterNode(mApp, VP_RENDER_GEOMETRY, original->getMatter()->getMaterial(), false, voxArray[k]));
                ((MatterNode*)matterNode.get())->setOffset(0.0f, 0.0f, i*20.0f + k *100.0f);
                mApp->getSceneGraph()->getRoot()->addChild(matterNode);
           }

           //Delete old one.
           original->deleteNode();
        }
    }

    //Finished
    mEnergyGrids.clear(); //Note: Good optimisation would not discard the energy grids that haven't changed, simply reset their energy.
}
