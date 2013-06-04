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
    Manages all the sub-parts of a Matter Object.
    Including:
        - The Voxel Field
        - The Rigid Body
        - The Vertex Shell
*/
class Matter
{
    public:
        Matter(App* app, MaterialPtr material, bool floating);
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

        //Setup collision hulls for this Matter Object.
        void setupHulls();

        void addHullVertex(unsigned int hullIndex, Vector3f vertex); //Add a hull point to be drawn in debug rendering.
        void addVoxelVertex(unsigned int hullIndex, Vector3f vertex); //Add a voxel point to be drawn in debug rendering.
        void addPressureVertex(float pressure, float stress, float strength, Vector3f vertex); //Add a pressure point to be drawn in debug rendering.
        void addEnergyBridge(Vector3i vertex, bool local); //Add an energy bridge to be drawn in debug rendering.

        //Signal that hulls have finished being processed.
        void hullsDone();

        //Get the Bullet Physics convex hull.
        btConvexHullShape* getHull(int index);

        VoxelField* getVoxelField();
        VertexShell* getVertexShell();
        RigidBody* getRigidBody();

        const GLfloat* getColorPtr() const;

        //Import voxel field from a file.
        void import(const char* voxelFilename);

        //Render voxels as dots.
        void debugRenderVoxels();

        //Render convex hulls as dots.
        void debugRenderHulls();

        void setupPressureRendering(); //Initialise the buffers and other stuff needed to perform debug rendering.
        void clearPressureRendering(); //Reset debug rendering data.
        void debugRenderPressure(); //Render pressure data.

        //bool floats();

        void setCenterOfMass(Vector3f cOfM);

        //Set the starting position of the object once it starts being simulated.
        //This must be called before creating the rigid body.
        void setStartingPosition(Vector3f offset);

        //Start feeding in vertices, for CPU processing.
        void beginProcessing();

        //Stop feeding in vertices and create the rigidbody.
        void endProcessing(MatterNode* node, bool usingCPU);

        void setCollided(bool value);
        bool hasCollided();

        //File used to create this object.
        std::string mFilename;

        //Deprecated debug method
        void processCollision(const btVector3& point, const btVector3& normal, const btScalar force);

        //Return unique ID of this matter object.
        unsigned int getID();

        //Static method to get new unique ID's.
        static unsigned int gCurrentMatterID;
        static unsigned int getNewID();

        void setMaterial(MaterialPtr material);
        MaterialPtr getMaterial();

        float getMass();

        //Check if the amount of energy can be considered trivial to this matter object.
        bool nonTrivial(float energy);

    protected:
        unsigned int mMatterID; //Unique Identifier.

        bool mFloating;
        VoxelField mVoxelField;
        VertexShell mVertexShell;
        RigidBody mRigidBody;

        //GLfloat mColor[3];

        std::vector<btConvexHullShape*> mCollisionHulls;

        //Debug rendering data
        std::vector<GLuint> m_debug_VoxelBufferIDs;
        std::vector<std::vector<GLfloat>> m_debug_VoxelVertexArrays;

        std::vector<GLuint> m_debug_HullBufferIDs;
        std::vector<std::vector<GLfloat>> m_debug_HullVertexArrays;

        GLuint m_debug_PressureVertexBufferID;
        std::vector<GLfloat> m_debug_PressureVertexArray; //Interleaved, 3 coords then 3 colors.

        int mRigidBodyMass;
        Vector3f mRigidBodyCenterOfMass;
        Vector3f mStartingWorldPosition;

        bool mCollided;

        MaterialPtr mMaterial; //Properties

    private:
};

#endif // MATTER_H
