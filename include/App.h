#ifndef APP_H
#define APP_H

#include "VP.h"

class Renderer;
class ResourceManager;
class PhysicsManager;
class SceneGraph;
class VoxelConverter;
class DestructionEngine;

class App
{
    public:
        App();
        virtual ~App();

        void init();

        Renderer* getRenderer();
        PhysicsManager* getPhysicsManager();
        ResourceManager* getResourceManager();
        SceneGraph* getSceneGraph();
        VoxelConverter* getVoxelConverter();
        DestructionEngine* getDestructionEngine();


        //Use the CPU for voxel convserion.
        void useCPUforVoxelConversion(GLboolean setting);

        //Set the frequnecy of ticks, in seconds.
        void setTickFrequency(GLdouble tickFreq);

        //Set the frequency of frames, in seconds.
        void setFrameFrequency(GLdouble frameFreq);

        //Register that simulation has started
        void clockTick();

        //Register that rendering has started
        void clockFrame();
        GLdouble deltaTime();

        //Returns true if it is time to render another frame.
        GLboolean timeToRender();

        //Returns true if it is time to simulate another tick.
        GLboolean timeToSimulate();

        //Get the amount of time the app can sleep.
        GLdouble sleepTime();

        //Input Control
        GLboolean gMoveLeft;
        GLboolean gMoveRight;
        GLboolean gMoveForward;
        GLboolean gMoveBackward;
        GLboolean gMoveUp;
        GLboolean gMoveDown;

        GLboolean gYawLeft;
        GLboolean gYawRight;
        GLboolean gPitchUp;
        GLboolean gPitchDown;
        GLboolean gRollLeft;
        GLboolean gRollRight;


    protected:

        Renderer* mRenderer;
        ResourceManager* mResourceManager;
        PhysicsManager* mPhysicsManager;
        SceneGraph* mSceneGraph;
        VoxelConverter* mVoxelConverter;
        DestructionEngine* mDestructionEngine;

        GLdouble mLastFrameTime;
        GLdouble mLastTickTime;
        GLdouble mLastSimulationTime;
        GLdouble mTickFreq;
        GLdouble mFrameFreq;

        GLint mTickCount;
        GLint mFrameCount;

        GLboolean mVoxelConversionCPU;



    private:
};

#endif // APP_H
