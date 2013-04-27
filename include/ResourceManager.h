#ifndef RESOURSEMANAGER_H
#define RESOURSEMANAGER_H

#include <string>
#include <vector>

#include "Mesh.h"

class VoxelField;

//Class: ResourseManager
//Description: Loads and stores resources or references to resources such as meshes or textures.
//Currently do not load meshes from file or bother with texturing.
class ResourceManager
{
    public:
        ResourceManager();
        virtual ~ResourceManager();

        GLint addMesh(Mesh mesh); //Adds a mesh to the Mesh Cache, returns the meshes index.
        Mesh* getMesh(GLint index); //Return a pointer mesh at the given index.
        GLint loadVoxelField(const char* filename);

        static ResourceManager theResourceManager;

    protected:
        std::vector<Mesh> mMeshCache;
        std::vector<GLint> mTextureIndices;
        std::vector<VoxelField> mVoxelCache;

    private:
};

#endif // RESOURSEMANAGER_H
