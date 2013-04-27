#ifndef VOXELDECOMPOSER_H
#define VOXELDECOMPOSER_H

#include "VoxelField.h"

enum voxDecompCode
{
    VOX_DECOMP_FAIL,
    VOX_DECOMP_FAIL_ALL,
    VOX_DECOMP_OK
};

class VoxelDecomposer
{
    public:
        VoxelDecomposer();
        virtual ~VoxelDecomposer();

        void decompose(VoxelField* vox);

        int walk(int i);
        int strafe(int i);

        /*
            Class to manage blocks during convex decomposition.
        */
        class Block
        {
            public:
                unsigned int x;
                unsigned int y;
                unsigned int z;
                const VoxelField* voxelData;
                unsigned int hullID;

                Block(VoxelField* data, unsigned int hull, unsigned int xPos, unsigned int yPos, unsigned int zPos)
                {
                    voxelData = data;
                    hullID = hull;
                    x = xPos;
                    y = yPos;
                    z = zPos;
                    pZ = nZ = pX = nX = pY = nY = -1;
                    canExpand = false;
                }

                //These 6 ints represent the face value of the blocks during the algorithm.
                // -1 : Unkown
                // -2 : SHOULD be another block here. Check before expanding.
                // -3 : Guaranteed to be another block here. Don't expand.
                // 0 : Can't expand here, but children might.
                // 1 : Can expand 1 time.
                // 2 : Can't expand here, and children cannot also.
                // 3 : MUST make sure can expand here before expanding.
                int pZ; //positive z, or up.
                int nZ; //negative z, or down.
                int pX; //positive x.
                int nX; //negative x.
                int pY; //positive y.
                int nY; //negative y.

                voxDecompCode expandPZ();
                voxDecompCode expandNZ();
                voxDecompCode expandPX();
                voxDecompCode expandNX();
                voxDecompCode expandPY();
                voxDecompCode expandNY();

                bool canExpandTo(int x, int y, int z);
                bool canExpand;
                void preExpand();

                bool checkPZ();
                bool checkNZ();
                bool checkPX();
                bool checkNX();
                bool checkPY();
                bool checkNY();

                bool isEdgeX();
                bool isEdgeY();
                bool isEdgeZ();
        };

    protected:
    private:
};

#endif // VOXELDECOMPOSER_H
