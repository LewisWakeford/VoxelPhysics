#include "DestructionEngine.h"
#include "Matter.h"
#include "MatterNode.h"

#include "App.h"
#include "SceneGraph.h"

DestructionEngine::DestructionEngine(App* app)
{
    mApp = app;
    mDestructionsPerformed = 0;
}

DestructionEngine::~DestructionEngine()
{
    //dtor
}

void DestructionEngine::processSet(const MatterCollisionSet& set)
{
    mNumberOfNonTrivialCollisions = 0;
    for(int i = 0; i < set.numCollisions(); i++)
    {
        processCollision(set.get(i));
    }
    //std::cout << "   NONTRIVIAL: " << mNumberOfNonTrivialCollisions << std::endl;
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

    first->clearPressureRendering();
    second->clearPressureRendering();

    //Check that collision is not trivial:
    if(first->nonTrivial(energyFirstScalar + energySecondScalar) ||
        second->nonTrivial(energyFirstScalar + energySecondScalar))
    {
        //mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "\n\n****NON TRIVIAL***");
        unsigned int firstIndex = getGridIndex(collision.getFirst());
        unsigned int secondIndex = getGridIndex(collision.getSecond());

        Vector3f myEnergyFirst(energyFirst.x(), energyFirst.y(), energyFirst.z());
        Vector3f myEnergySecond(energySecond.x(), energySecond.y(), energySecond.z());

        mEnergyGrids[firstIndex].setEnergy(myEnergyFirst);
        mEnergyGrids[secondIndex].setEnergy(myEnergySecond);

        if(firstSpeed >= secondSpeed)
        {
            double preBridge = glfwGetTime();
            buildBridges(firstIndex, secondIndex, collision);
            double postBridge = glfwGetTime();
            mApp->debugPrint(App::DEBUG_BRIDGES, "Time to build bridges: ", (postBridge - preBridge));

            transferEnergy(mEnergyGrids[firstIndex], mEnergyGrids[secondIndex]);
        }
        else
        {
            double preBridge = glfwGetTime();
            buildBridges(secondIndex, firstIndex, collision);
            double postBridge = glfwGetTime();
            mApp->debugPrint(App::DEBUG_BRIDGES, "Time to build bridges: ", (postBridge - preBridge));

            transferEnergy(mEnergyGrids[secondIndex], mEnergyGrids[firstIndex]);
        }
        mNumberOfNonTrivialCollisions++;
    }
}

void DestructionEngine::transferEnergy(EnergyGrid& first, EnergyGrid& second)
{
    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Starting Transfer...");
    first.setCollisionPartner(&second);
    second.setCollisionPartner(&first);

    const Vector3f& firstEnergy = first.getEnergyVector();
    const Vector3f& secondEnergy = second.getEnergyVector();

    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Building Maps...");
    first.buildMaps(secondEnergy);
    second.buildMaps(firstEnergy);


    //Perform First's transfer into Second.

    //Calculate seconds energy in terms of first
    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing setup...");
    Vector3f directionFirst = firstEnergy.normalized();
    float secondEnergyRelative = directionFirst.dot(secondEnergy);
    float firstEnergyScalar = firstEnergy.length();
    float secondEnergyScalar = secondEnergy.length();

    unsigned int totalVoxels = first.getMatter()->getVoxelField()->getNumVoxels() + second.getMatter()->getVoxelField()->getNumVoxels();

    if(first.getMatter()->nonTrivial(firstEnergyScalar) || second.getMatter()->nonTrivial(firstEnergyScalar))
    {
        float energyPerVoxel = (firstEnergyScalar + secondEnergyRelative) / totalVoxels;

        first.setEnergyPerVoxel(energyPerVoxel);
        second.setEnergyPerVoxel(energyPerVoxel);

        first.setAsProjector();
        second.setAsReciever();


        first.setInitialEnergy(firstEnergyScalar);
        second.setInitialEnergy(secondEnergyRelative);

        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing direct transfer...");
        double directTransferStart = glfwGetTime();
        first.directTransfer();
        double directTransferEnd = glfwGetTime();
        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Time taken: ", (directTransferEnd - directTransferStart));

        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing indirect transfer...");
        first.indirectTransfer();

        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing indirect transfer...");
        second.indirectTransfer();
    }

    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing setup...");
    //Do the same for second

    float remainingEnergyRatio = secondEnergyRelative / secondEnergyScalar;
    Vector3f remainingEnergy = secondEnergy.mult(remainingEnergyRatio);
    float remainingEnergyScalar = remainingEnergy.length();

    //Vector3f directionSecond = remainingEnergy.normalized();
    //float firstEnergyRelative = directionSecond.dot(firstEnergy);



    if(first.getMatter()->nonTrivial(remainingEnergyScalar) || second.getMatter()->nonTrivial(remainingEnergyScalar))
    {
        float energyPerVoxel = (remainingEnergyScalar) / totalVoxels;

        first.setEnergyPerVoxel(energyPerVoxel);
        second.setEnergyPerVoxel(energyPerVoxel);

        second.setAsProjector();
        first.setAsReciever();

        first.setInitialEnergy(0.0f);
        second.setInitialEnergy(remainingEnergyScalar);

        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing direct transfer...");
        double directTransferStart = glfwGetTime();
        second.directTransfer();
        double directTransferEnd = glfwGetTime();
        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Time taken: ", (directTransferEnd - directTransferStart));

        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing indirect transfer...");
        second.indirectTransfer();

        mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Performing indirect transfer...");
        first.indirectTransfer();
    }

    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "---SET---");
    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "First: ", firstEnergyScalar);
    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Second: ", secondEnergyRelative);


    first.updateRenderData();
    second.updateRenderData();
    first.getMatter()->setupPressureRendering();
    second.getMatter()->setupPressureRendering();
}

void DestructionEngine::buildBridges(unsigned int firstIndex, unsigned int secondIndex, const MatterCollision& collision)
{
    mApp->debugPrint(App::DEBUG_INTERAL_SIMULATION, "Building Bridges...");

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
        //voxel.add((first.getEnergyVector().normalized()).mult(0.5));



        //Test if voxel is within bounding box of the other matter.
        if(secondBB.pointInside(voxel))
        {

            closeBridgesFirst.push_back(voxel);
            closeBridgesFirstOriginal.push_back(i);
        }
    }

    Vector3f cOfMS = second.getMatter()->getVoxelField()->getCenterOfMass();
    for(int i = 0; i < potentialBridgesSecond.size(); i++)
    {
        //Transform each voxel into world space.
        Vector3f voxel = secondTransform.transformVertex(Vector3f(potentialBridgesSecond[i].x-cOfMS.x, potentialBridgesSecond[i].y-cOfMS.y, potentialBridgesSecond[i].z-cOfMS.z));
        //voxel.add((-(first.getEnergyVector()).normalized()).mult(0.5));



        //Test if voxel is within bounding box of the other matter.
        if(firstBB.pointInside(voxel))
        {

            closeBridgesSecond.push_back(voxel);
            closeBridgesSecondOriginal.push_back(i);
        }
    }

    float closeRadius = 1.3f;

    //Perform more advanced tests.
    for(int i = 0; i < closeBridgesFirst.size(); i++)
    {
        for(int k = 0; k < closeBridgesSecond.size(); k++)
        {
            float distance = closeBridgesFirst[i].distance(closeBridgesSecond[k]);
            if(distance < closeRadius)
            {
                //Vector3f localPointFirst = firstTransformInverse.transformVertex(closeBridgesSecond[i]) + cOfMF;
                //Vector3f localPointSecond  = secondTransformInverse.transformVertex(closeBridgesFirst[i]) + cOfMS;
                first.addBridge(potentialBridgesFirst[closeBridgesFirstOriginal[i]], potentialBridgesSecond[closeBridgesSecondOriginal[k]]);
                second.addBridge(potentialBridgesSecond[closeBridgesSecondOriginal[k]], potentialBridgesFirst[closeBridgesFirstOriginal[i]]);
            }
        }
    }
}

void DestructionEngine::checkForSeparation()
{
    for(unsigned int i = 0; i < mEnergyGrids.size(); i++)
    {
        MatterNode* original = mEnergyGrids[i].getMatterNode();
        Vector3f originalCofM = original->getMatter()->getVoxelField()->getCenterOfMass();
        std::vector<VoxelField> voxArray;
        std::vector<float> energyR;
        std::vector<float> energyP;
        double startDestruction = glfwGetTime();
        bool breakage = mEnergyGrids[i].separate(voxArray, energyR, energyP);
        double endDestruction = glfwGetTime();
        mApp->debugPrint(App::DEBUG_BREAKING, "Time to destroy: ", (endDestruction - startDestruction));

        if(breakage)
        {
            mDestructionsPerformed++;
            //Create new matter nodes.
            mApp->debugPrint(App::DEBUG_BREAKING, "BREAK!");

            for(unsigned int k = 0; k < voxArray.size(); k++)
            {
               //Get posistion of new voxel field.
               Vector3f CofM = voxArray[k].getCenterOfMass();
               Vector3f offset = CofM - originalCofM;
               Matrix4D transform = original->getTransform();
               transform.translate(offset.x, offset.y, offset.z);

               //Set Energy
               Vector3f energyVector = mEnergyGrids[i].constructEnergyVector(energyP[k], energyR[k]);

               MatterNodePtr matterNode(new MatterNode(mApp, VP_RENDER_GEOMETRY, original->getMatter()->getMaterial(), false, voxArray[k]));
                matterNode->setTransform(transform);
                matterNode->setEnergy(energyVector);
                mApp->getSceneGraph()->getRoot()->addChild(matterNode);

            }

            //Delete old one.
            original->deleteNode();
            double processing = glfwGetTime();
            mApp->debugPrint(App::DEBUG_BREAKING, "Time to process: ", (processing - endDestruction));
        }

    }

    //Finished
    mEnergyGrids.clear(); //Note: Good optimisation would not discard the energy grids that haven't changed, simply reset their energy.
}
