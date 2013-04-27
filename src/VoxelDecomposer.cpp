#include "VoxelDecomposer.h"

#include <vector>

VoxelDecomposer::VoxelDecomposer()
{
    //ctor
}

VoxelDecomposer::~VoxelDecomposer()
{
    //dtor
}

voxDecompCode VoxelDecomposer::Block::expandPX()
{
    if(!canExpand) return VOX_DECOMP_FAIL;
    //Cannot expand onto a block that already exists.
    if(pX < -1) return VOX_DECOMP_FAIL;
    if(pX == 0) return VOX_DECOMP_FAIL;
    if(pX == 2) return VOX_DECOMP_FAIL;

    //Check sides. If there SHOULD be a block in these slots but there isn't
    /* Actually, X expansion doesn't care.
    if(!checkPZ() || !checkNZ() || !checkPY() || !checkNY())
    {
        pX = 0;
        return VOX_DECOMP_FAIL;
    }
    */

    int failNumber;
    if(pX == 1) failNumber = 2;
    if(pX == -1) failNumber = 0;

    //Check what is in the expandsion slot:
    int targetX = x + 1;
    if(targetX > 31)
    {
        pX = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check there is no other hull
    int expansionHull = voxelData->hullAt(targetX, y, z);
    if(expansionHull != -1 && expansionHull != hullID)
    {
        pX = failNumber;
        return VOX_DECOMP_FAIL;
    }

    GLubyte expansionVoxel = voxelData->get(targetX, y, z);

    switch(expansionVoxel)
    {
        case 0:
            pX = failNumber;
            if(isEdgeX())
            {
                return VOX_DECOMP_FAIL_ALL;
            }
            else
            {
                return VOX_DECOMP_FAIL;
            }
            break;
        case 1:
            return VOX_DECOMP_OK;
            break;
    }

}

voxDecompCode VoxelDecomposer::Block::expandNX()
{
    if(!canExpand) return VOX_DECOMP_FAIL;
    //Cannot expand onto a block that already exists.
    if(nX < -1) return VOX_DECOMP_FAIL;
    if(nX == 0) return VOX_DECOMP_FAIL;
    if(nX == 2) return VOX_DECOMP_FAIL;

    //Check sides. If there SHOULD be a block in these slots but there isn't
    /*
    if(!checkPZ() || !checkNZ() || !checkPY() || !checkNY())
    {
        nX = 0;
        return VOX_DECOMP_FAIL;
    }
    */

    int failNumber;
    if(nX == 1) failNumber = 2;
    if(nX == -1) failNumber = 0;

    //Check what is in the expansion slot:
    int targetX = x - 1;
    if(targetX < 0)
    {
        nX = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check there is no other hull
    int expansionHull = voxelData->hullAt(targetX, y, z);
    if(expansionHull != -1 && expansionHull != hullID)
    {
        nX = failNumber;
        return VOX_DECOMP_FAIL;
    }

    GLubyte expansionVoxel = voxelData->get(targetX, y, z);

    switch(expansionVoxel)
    {
        case 0:
            nX = failNumber;
            if(isEdgeX())
            {
                return VOX_DECOMP_FAIL_ALL;
            }
            else
            {
                return VOX_DECOMP_FAIL;
            }
            break;
        case 1:
            return VOX_DECOMP_OK;
            break;
    }

}

voxDecompCode VoxelDecomposer::Block::expandPY()
{
    if(!canExpand) return VOX_DECOMP_FAIL;
    //Cannot expand onto a block that already exists.
    if(pY < -1) return VOX_DECOMP_FAIL;
    if(pY == 0) return VOX_DECOMP_FAIL;
    if(pY == 2) return VOX_DECOMP_FAIL;

    int failNumber;
    if(pY == 1) failNumber = 2;
    if(pY == -1) failNumber = 0;

    //Check that the X expansion results make sense.
    if(!checkPX() || !checkNX())
    {
        pY = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check what is in the expandsion slot:
    int targetY = y + 1;
    if(targetY > 31)
    {
        pY = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check there is no other hull
    int expansionHull = voxelData->hullAt(x, targetY, z);
    if(expansionHull != -1 && expansionHull != hullID)
    {
        pY = failNumber;
        return VOX_DECOMP_FAIL;
    }

    GLubyte expansionVoxel = voxelData->get(x, targetY, z);

    switch(expansionVoxel)
    {
        case 0:
            pY = failNumber;
            if(isEdgeY())
            {
                return VOX_DECOMP_FAIL_ALL;
            }
            else
            {
                return VOX_DECOMP_FAIL;
            }
            break;
        case 1:
            return VOX_DECOMP_OK;
            break;
    }
}

voxDecompCode VoxelDecomposer::Block::expandNY()
{
    if(!canExpand) return VOX_DECOMP_FAIL;
    //Cannot expand onto a block that already exists.
    if(nY < -1) return VOX_DECOMP_FAIL;
    if(nY == 0) return VOX_DECOMP_FAIL;
    if(nY == 2) return VOX_DECOMP_FAIL;

    int failNumber;
    if(nY == 1) failNumber = 2;
    if(nY == -1) failNumber = 0;

    //Check sides. If there SHOULD be a block in these slots but there isn't
    if(!checkPX() || !checkNX())
    {
        nY = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check what is in the expandsion slot:
    int targetY = y - 1;
    if(targetY < 0)
    {
        nY = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check there is no other hull
    int expansionHull = voxelData->hullAt(x, targetY, z);
    if(expansionHull != -1 && expansionHull != hullID)
    {
        nY = failNumber;
        return VOX_DECOMP_FAIL;
    }

    GLubyte expansionVoxel = voxelData->get(x, targetY, z);

    switch(expansionVoxel)
    {
        case 0:
            nY = failNumber;
            if(isEdgeY())
            {
                return VOX_DECOMP_FAIL_ALL;
            }
            else
            {
                return VOX_DECOMP_FAIL;
            }
            break;
        case 1:
        default:
            return VOX_DECOMP_OK;
            break;
    }
}

voxDecompCode VoxelDecomposer::Block::expandPZ()
{
    if(!canExpand) return VOX_DECOMP_FAIL;
    //Cannot expand onto a block that already exists.
    if(pZ < -1) return VOX_DECOMP_FAIL;
    if(pZ == 0) return VOX_DECOMP_FAIL;
    if(pZ == 2) return VOX_DECOMP_FAIL;

    int failNumber;
    if(pZ == 1) failNumber = 2;
    if(pZ == -1) failNumber = 0;

    //Check sides. If there SHOULD be a block in these slots but there isn't
    if(!checkPY() || !checkNY() || !checkPX() || !checkNX())
    {
        pZ = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check what is in the expandsion slot:
    int targetZ = z + 1;
    if(targetZ > 31)
    {
        pZ = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check there is no other hull
    int expansionHull = voxelData->hullAt(x, y, targetZ);
    if(expansionHull != -1 && expansionHull != hullID)
    {
        pZ = failNumber;
        return VOX_DECOMP_FAIL;
    }

    GLubyte expansionVoxel = voxelData->get(x, y, targetZ);


    switch(expansionVoxel)
    {
        case 0:
            pZ = failNumber;
            if(isEdgeZ())
            {
                return VOX_DECOMP_FAIL_ALL;
            }
            else
            {
                return VOX_DECOMP_FAIL;
            }
            break;
        case 1:
            return VOX_DECOMP_OK;
            break;
    }
}

voxDecompCode VoxelDecomposer::Block::expandNZ()
{
    if(!canExpand) return VOX_DECOMP_FAIL;
    //Cannot expand onto a block that already exists.
    if(nZ < -1) return VOX_DECOMP_FAIL;
    if(nZ == 0) return VOX_DECOMP_FAIL;
    if(nZ == 2) return VOX_DECOMP_FAIL;

    int failNumber;
    if(nZ == 1) failNumber = 2;
    if(nZ == -1) failNumber = 0;

    //Check sides. If there SHOULD be a block in these slots but there isn't
    if(!checkPY() || !checkNY() || !checkPX() || !checkNX())
    {
        nZ = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check what is in the expandsion slot:
    int targetZ = z - 1;
    if(targetZ < 0)
    {
        nZ = failNumber;
        return VOX_DECOMP_FAIL;
    }

    //Check there is no other hull
    int expansionHull = voxelData->hullAt(x, y, targetZ);
    if(expansionHull != -1 && expansionHull != hullID)
    {
        nZ = failNumber;
        return VOX_DECOMP_FAIL;
    }

    GLubyte expansionVoxel = voxelData->get(x, y, targetZ);


    switch(expansionVoxel)
    {
        case 0:
            nZ = failNumber;
            if(isEdgeZ())
            {
                return VOX_DECOMP_FAIL_ALL;
            }
            else
            {
                return VOX_DECOMP_FAIL;
            }
            break;
        case 1:
            return VOX_DECOMP_OK;
            break;
    }
}

bool VoxelDecomposer::Block::checkPZ()
{
    if(pZ == -3) return true;
    if(pZ == -2)
    {
        unsigned int targetZ = z + 1;
        if(targetZ > 31) return false;
        GLuint voxel = voxelData->hullAt(x, y, targetZ);
        if(voxel == hullID)
        {
            pZ = -3;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool VoxelDecomposer::Block::checkNZ()
{
    if(nZ == -3) return true;
    if(nZ == -2)
    {
        unsigned int targetZ = z - 1;
        if(targetZ < 0) return false;
        GLuint voxel = voxelData->hullAt(x, y, targetZ);
        if(voxel == hullID)
        {
            nZ = -3;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool VoxelDecomposer::Block::checkPY()
{
    if(pY == -3) return true;
    if(pY == -2)
    {
        unsigned int targetY = y + 1;
        if(targetY > 31) return false;
        GLuint voxel = voxelData->hullAt(x, targetY, z);
        if(voxel == hullID)
        {
            pY = -3;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool VoxelDecomposer::Block::checkNY()
{
    if(nY == -3) return true;
    if(nY == -2)
    {
        unsigned int targetY = y - 1;
        if(targetY < 0) return false;
        GLuint voxel = voxelData->hullAt(x, targetY, z);
        if(voxel == hullID)
        {
            nY = -3;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool VoxelDecomposer::Block::checkPX()
{
    if(pX == -3) return true;
    if(pX == -2)
    {
        unsigned int targetX = x + 1;
        if(targetX > 31) return false;
        GLuint voxel = voxelData->hullAt(targetX, y, z);
        if(voxel == hullID)
        {
            pX = -3;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool VoxelDecomposer::Block::checkNX()
{
    if(nX == -3) return true;
    if(nX == -2)
    {
        unsigned int targetX = x - 1;
        if(targetX < 0) return false;
        GLuint voxel = voxelData->hullAt(targetX, y, z);
        if(voxel == hullID)
        {
            nX = -3;
            return true;
        }
        else
        {
            return false;
        }
    }
    return true;
}

bool VoxelDecomposer::Block::isEdgeX()
{
    bool isEdgeInY = true;
    bool isEdgeInZ = true;

    if (pY < -1 && nY < -1) isEdgeInY = !(checkPY() && checkNY());
    if (pZ < -1 && nZ < -1) isEdgeInZ = !(checkPZ() && checkNZ());


    return !(isEdgeInY || isEdgeInZ);
}

bool VoxelDecomposer::Block::isEdgeY()
{
    bool isEdgeInX = true;
    bool isEdgeInZ = true;

    if (pX < -1 && nX < -1) isEdgeInX = !(checkPX() && checkNX());
    if (pZ < -1 && nZ < -1) isEdgeInZ = !(checkPZ() && checkNZ());

    return !(isEdgeInX || isEdgeInZ);
}

bool VoxelDecomposer::Block::isEdgeZ()
{
    bool isEdgeInY = true;
    bool isEdgeInX = true;

    if (pY < -1 && nY < -1) isEdgeInY = !(checkPY() && checkNY());
    if (pX < -1 && nX < -1) isEdgeInX = !(checkPX() && checkNX());


    return !(isEdgeInY || isEdgeInX);
}


int VoxelDecomposer::walk(int i)
{
    if(i == -1) return i;
    if(i == 1) return 0;
    if(i == 3) return -1;
    return -3;
}

int VoxelDecomposer::strafe(int i)
{
    if(i == -1 || i == -2 || i == 2 || i == 3) return i;
    if(i == -3) return -2;
    if(i == 0 || i == 1) return 1;
    return -3;
}

bool VoxelDecomposer::Block::canExpandTo(int x, int y, int z)
{
    if(x < 0 || x > 31 || y < 0 || y > 31 || z < 0 || z > 31) return false;

    int expansionHull = voxelData->hullAt(x, y, z);

    if(expansionHull != -1 && expansionHull != hullID) return false;

    GLubyte expansionVoxel = voxelData->get(x, y, z);

    if(expansionVoxel != 1) return false;
    else return true;
}

void VoxelDecomposer::Block::preExpand()
{
    canExpand = true;

    if(nX == -1 || nX == 1)
    {
        if(canExpandTo(x-1, y, z))
        {
            nX = 3;
        }
    }
    else if(nX == 3)
    {
        if(!canExpandTo(x-1, y, z))
        {
            canExpand = false;
        }
    }

    if(pX == -1 || pX == 1)
    {
        if(canExpandTo(x+1, y, z))
        {
            pX = 3;
        }
    }
    else if(pX == 3)
    {
        if(!canExpandTo(x+1, y, z))
        {
            canExpand = false;
        }
    }

    if(nY == -1 || nY == 1)
    {
        if(canExpandTo(x, y-1, z))
        {
            nY = 3;
        }
    }
    else if(nY == 3)
    {
        if(!canExpandTo(x, y-1, z))
        {
            canExpand = false;
        }
    }

    if(pY == -1 || pY == 1)
    {
        if(canExpandTo(x, y+1, z))
        {
            pY = 3;
        }
    }
    else if(pY == 3)
    {
        if(!canExpandTo(x, y+1, z))
        {
            canExpand = false;
        }
    }

    if(nZ == -1 || nZ == 1)
    {
        if(canExpandTo(x, y, z-1))
        {
            nZ = 3;
        }
    }
    else if(nZ == 3)
    {
        if(!canExpandTo(x, y, z-1))
        {
            canExpand = false;
        }
    }

    if(pZ == -1 || pZ == 1)
    {
        if(canExpandTo(x, y, z+1))
        {
            pZ = 3;
        }
    }
    else if(pZ == 3)
    {
        if(!canExpandTo(x, y, z+1))
        {
            canExpand = false;
        }
    }
}

/*
    Next thing to do, decide on a direction on the fly. IE if direction is ambigueous at start, keep going until can be discerned.
*/
void VoxelDecomposer::decompose(VoxelField* vox)
{
    unsigned int hullID = 0;
    std::vector<Block> blocksInHull;
    std::vector<Block> tempBlocks;

    unsigned int consumedVoxels = 0;
    unsigned int totalVoxels = vox->getNumVoxels();

    while(consumedVoxels < totalVoxels)
    {
        //Create a new shape.
        blocksInHull.clear();
        tempBlocks.clear();

        //First unocuppied voxel, starting at the top (z) and moving down.
        Vector3i start = vox->getFirstFree();
        Block firstBlock(vox, hullID, start.x, start.y, start.z);
        firstBlock.preExpand();
        blocksInHull.push_back(firstBlock);
        vox->setHull(firstBlock.z, firstBlock.y, firstBlock.x, hullID);
        consumedVoxels++;

        bool moveX = (firstBlock.pX == 3);
        bool moveY = (firstBlock.pY == 3);

        bool shapeDone = false;

        while(!shapeDone)
        {
            int voxelsAtStartOfExpansion = consumedVoxels;
            int successfulExpansions = 0;

if(!moveX)
{
            //Expand negative in X axis
            successfulExpansions = 0;
            for(unsigned int i = 0; i < blocksInHull.size(); i++)
            {
                Block& currentBlock = blocksInHull[i];
                voxDecompCode code = currentBlock.expandNX();
                if(code == VOX_DECOMP_OK)
                {
                    successfulExpansions++;
                    Block newBlock(vox, hullID, currentBlock.x-1, currentBlock.y, currentBlock.z);
                    newBlock.nX = walk(currentBlock.nX);
                    newBlock.pX = -3;
                    newBlock.nY = strafe(currentBlock.nY);
                    newBlock.pY = strafe(currentBlock.pY);
                    newBlock.nZ = strafe(currentBlock.nZ);
                    newBlock.pZ = strafe(currentBlock.pZ);

                    currentBlock.nX = -3;

                    tempBlocks.push_back(newBlock);
                }
                else if(code == VOX_DECOMP_FAIL_ALL)
                {
                    if(successfulExpansions > 0)
                    {
                        shapeDone = true;
                    }
                }
            }

            if(shapeDone)
            {
                break;
            }
            else
            {
                for(unsigned int i = 0; i < tempBlocks.size(); i++)
                {
                    tempBlocks[i].preExpand();
                    blocksInHull.push_back(tempBlocks[i]);
                    vox->setHull(tempBlocks[i].z, tempBlocks[i].y, tempBlocks[i].x, hullID);
                    consumedVoxels++;
                }
                tempBlocks.clear();
            }
}
else
{
            //Expand positive in X axis
            successfulExpansions = 0;
            for(unsigned int i = 0; i < blocksInHull.size(); i++)
            {
                Block& currentBlock = blocksInHull[i];
                voxDecompCode code = currentBlock.expandPX();
                if(code == VOX_DECOMP_OK)
                {
                    successfulExpansions++;
                    Block newBlock(vox, hullID, currentBlock.x+1, currentBlock.y, currentBlock.z);
                    newBlock.pX = walk(currentBlock.pX);
                    newBlock.nX = -3;
                    newBlock.nY = strafe(currentBlock.nY);
                    newBlock.pY = strafe(currentBlock.pY);
                    newBlock.nZ = strafe(currentBlock.nZ);
                    newBlock.pZ = strafe(currentBlock.pZ);

                    currentBlock.pX = -3;

                    tempBlocks.push_back(newBlock);
                }
                else if(code == VOX_DECOMP_FAIL_ALL)
                {
                    if(successfulExpansions > 0)
                    {
                        shapeDone = true;
                    }
                }
            }

            if(shapeDone)
            {
                break;
            }
            else
            {
                for(int i = 0; i < tempBlocks.size(); i++)
                {
                    tempBlocks[i].preExpand();
                    blocksInHull.push_back(tempBlocks[i]);
                    vox->setHull(tempBlocks[i].z, tempBlocks[i].y, tempBlocks[i].x, hullID);
                    consumedVoxels++;
                }
                tempBlocks.clear();
            }
}
if(!moveY)
{
            //Expand negative in Y axis
            successfulExpansions = 0;
            for(int i = 0; i < blocksInHull.size(); i++)
            {
                Block& currentBlock = blocksInHull[i];
                voxDecompCode code = currentBlock.expandNY();
                if(code == VOX_DECOMP_OK)
                {
                    successfulExpansions++;
                    Block newBlock(vox, hullID, currentBlock.x, currentBlock.y-1, currentBlock.z);
                    newBlock.nX = strafe(currentBlock.nX);
                    newBlock.pX = strafe(currentBlock.pX);
                    newBlock.nY = walk(currentBlock.nY);
                    newBlock.pY = -3;
                    newBlock.nZ = strafe(currentBlock.nZ);
                    newBlock.pZ = strafe(currentBlock.pZ);

                    currentBlock.nY = -3;

                    tempBlocks.push_back(newBlock);
                }
                else if(code == VOX_DECOMP_FAIL_ALL)
                {
                    if(successfulExpansions > 0)
                    {
                        shapeDone = true;
                    }
                }
            }

            if(shapeDone)
            {
                break;
            }
            else
            {
                for(int i = 0; i < tempBlocks.size(); i++)
                {
                    tempBlocks[i].preExpand();
                    blocksInHull.push_back(tempBlocks[i]);
                    vox->setHull(tempBlocks[i].z, tempBlocks[i].y, tempBlocks[i].x, hullID);
                    consumedVoxels++;
                }
                tempBlocks.clear();
            }
}
else{
            //Expand positive in Y axis
            successfulExpansions = 0;
            for(int i = 0; i < blocksInHull.size(); i++)
            {
                Block& currentBlock = blocksInHull[i];
                voxDecompCode code = currentBlock.expandPY();
                if(code == VOX_DECOMP_OK)
                {
                    successfulExpansions++;
                    Block newBlock(vox, hullID, currentBlock.x, currentBlock.y+1, currentBlock.z);
                    newBlock.pY = walk(currentBlock.pY);
                    newBlock.nY = -3;
                    newBlock.nX = strafe(currentBlock.nX);
                    newBlock.pX = strafe(currentBlock.pX);
                    newBlock.nZ = strafe(currentBlock.nZ);
                    newBlock.pZ = strafe(currentBlock.pZ);

                    currentBlock.pY = -3;

                    tempBlocks.push_back(newBlock);
                }
                else if(code == VOX_DECOMP_FAIL_ALL)
                {
                    if(successfulExpansions > 0)
                    {
                        shapeDone = true;
                    }
                }
            }

            if(shapeDone)
            {
                break;
            }
            else
            {
                for(int i = 0; i < tempBlocks.size(); i++)
                {
                    tempBlocks[i].preExpand();
                    blocksInHull.push_back(tempBlocks[i]);
                    vox->setHull(tempBlocks[i].z, tempBlocks[i].y, tempBlocks[i].x, hullID);
                    consumedVoxels++;
                }
                tempBlocks.clear();
            }
}
            //Expand Negative in Z axis
            successfulExpansions = 0;
            for(int i = 0; i < blocksInHull.size(); i++)
            {
                Block& currentBlock = blocksInHull[i];
                voxDecompCode code = currentBlock.expandNZ();
                if(code == VOX_DECOMP_OK)
                {
                    successfulExpansions++;
                    Block newBlock(vox, hullID, currentBlock.x, currentBlock.y, currentBlock.z-1);
                    newBlock.pY = strafe(currentBlock.pY);
                    newBlock.nY = strafe(currentBlock.nY);
                    newBlock.nX = strafe(currentBlock.nX);
                    newBlock.pX = strafe(currentBlock.pX);
                    newBlock.nZ = walk(currentBlock.nZ);
                    newBlock.pZ = -3;

                    currentBlock.nZ = -3;

                    tempBlocks.push_back(newBlock);
                }
                else if(code == VOX_DECOMP_FAIL_ALL)
                {
                    if(successfulExpansions > 0)
                    {
                        shapeDone = true;
                    }
                }
            }

            if(shapeDone)
            {
                break;
            }
            else
            {
                for(int i = 0; i < tempBlocks.size(); i++)
                {
                    tempBlocks[i].preExpand();
                    blocksInHull.push_back(tempBlocks[i]);
                    vox->setHull(tempBlocks[i].z, tempBlocks[i].y, tempBlocks[i].x, hullID);
                    consumedVoxels++;
                }
                tempBlocks.clear();
            }

            //If we didn't expand at all, shape is done:
            if(voxelsAtStartOfExpansion == consumedVoxels)
            {
                shapeDone = true;
            }
        }

        hullID++;
    }

    vox->setNumberOfHulls(hullID);
}
