#ifndef MATTER_H
#define MATTER_H

#include "VP.h"
#include "VoxelField.h"
#include "VertexShell.h"
#include "RigidBody.h"
#include "Material.h"

class App;

/**
    Class: Matter
    A Matter object links a voxel field with the vertex mesh generated from it.
*/
class Matter
{
    public:
        Matter(App* app, const Material*, bool floating);
        virtual ~Matter();

        /**
            Set the vertex field associated with this Matter
        */
        void setVoxelField(VoxelField voxelField);

        /**
            Add a vertex to the vertex shell of this Matter.
            Useful when generating the vertex shell.
        */
        void addVertex(GLfloat cX, GLfloat cY, GLfloat cZ, GLfloat nX, GLfloat nY, GLfloat nZ);
        void addVertex(Vector3f coordinates, Vector3f normal);
        void setupHulls();

        void addHullVertex(unsigned int hullIndex, Vector3f vertex); //Add a hull point to be drawn in debug rendering
        void addVoxelVertex(unsigned int hullIndex, Vector3f vertex); //Add a voxel point to be drawn in debug rendering
        void addPressureVertex(float pressure, float stress, float strength, Vector3f vertex); //Add a pressure point to be drawn in debug rendering
        void addEnergyBridge(Vector3f vertex, bool local);

        void hullsDone();
        btConvexHullShape* getHull(int index);

        VoxelField* getVoxelField();
        VertexShell* getVertexShell();
        RigidBody* getRigidBody();

        const GLfloat* getColorPtr() const;

        void import(const char* voxelFilename);

        void debugRenderVoxels();
        void debugRenderHulls();

        void setupPressureRendering();
        void clearPressureRendering();
        void debugRenderPressure();

        bool floats();

        void setCenterOfMass(Vector3 cOfM);

        void setStartingPosition(Vector3 offset);

        void beginProcessing();
        void endProcessing(MatterNode* node, bool usingCPU);

        void setCollided(bool value);
        bool hasCollided();

        std::string mFilename;

        void processCollision(const btVector3& point, const btVector3& normal, const btScalar force);

        unsigned int getID();

        static unsigned int gCurrentMatterID;

        static unsigned int getNewID();

        void setMaterial(const Material* material);
        const Material* getMaterial() const;

        float getMass();

        bool nonTrivial(float energy);

    protected:
        unsigned int mMatterID; //Unique Identifier.

        bool mFloating;
        VoxelField mVoxelField;
        VertexShell mVertexShell;
        RigidBody mRigidBody;

        GLfloat mColor[3];

        std::vector<btConvexHullShape*> mCollisionHulls;

        std::vector<GLuint> m_debug_VoxelBufferIDs;
        std::vector<std::vector<GLfloat>> m_debug_VoxelVertexArrays;

        std::vector<GLuint> m_debug_HullBufferIDs;
        std::vector<std::vector<GLfloat>> m_debug_HullVertexArrays;

        GLuint m_debug_PressureVertexBufferID;
        std::vector<GLfloat> m_debug_PressureVertexArray; //Interleaved, 3 coords then 3 colors.

        int mRigidBodyMass;
        Vector3 mRigidBodyCenterOfMass;
        Vector3 mStartingWorldPosition;

        bool mCollided;

        const Material* mMaterial; //Properties

    private:
};

#endif // MATTER_H
