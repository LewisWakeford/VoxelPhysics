#include <iostream>
#include "Vector3.h"

using namespace std;

int main()
{
    int corners = 0;

    Vector3f normal0(1.0f, 1.0f, 1.0f);
    Vector3f normal1(-1.0f, 1.0f, 1.0f);
    Vector3f normal2(1.0f, -1.0f, 1.0f);
    Vector3f normal3(-1.0f, -1.0f, 1.0f);
    Vector3f normal4(1.0f, 1.0f, -1.0f);
    Vector3f normal5(-1.0f, 1.0f, -1.0f);
    Vector3f normal6(1.0f, -1.0f, -1.0f);
    Vector3f normal7(-1.0f, -1.0f, -1.0f);

    for(int a = 0; a < 2; a++)
    {
        corners = corners ^ 1;
        for(int b = 0; b < 2; b++)
        {
            corners = corners ^ (1 << 1);
            for(int c = 0; c < 2; c++)
            {
                corners = corners ^ (1 << 2);
                for(int d = 0; d < 2; d++)
                {
                    corners = corners ^ (1 << 3);
                    for(int e = 0; e < 2; e++)
                    {
                        corners = corners ^ (1 << 4);
                        for(int f = 0; f < 2; f++)
                        {
                            corners = corners ^ (1 << 5);
                            for(int g = 0; g < 2; g++)
                            {
                                corners = corners ^ (1 << 6);
                                for(int h = 0; h < 2; h++)
                                {
                                    corners = corners ^ (1 << 7);

                                    Vector3f cubeNormal;
                                    if(corners & 1) cubeNormal.add(normal0);
                                    if(corners & 2) cubeNormal.add(normal1);
                                    if(corners & 4) cubeNormal.add(normal2);
                                    if(corners & 8) cubeNormal.add(normal3);
                                    if(corners & 16) cubeNormal.add(normal4);
                                    if(corners & 32) cubeNormal.add(normal5);
                                    if(corners & 64) cubeNormal.add(normal6);
                                    if(corners & 128) cubeNormal.add(normal7);
                                    cubeNormal.normalize();

                                    std::cout << "mCubeNormals[" << corners << "] = Vector3f (" << cubeNormal.x << ", " << cubeNormal.y << ", " << cubeNormal.z << ");" << std::endl;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return 0;
}
