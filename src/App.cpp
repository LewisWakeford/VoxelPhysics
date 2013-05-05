#include "App.h"
#include "Renderer.h"
#include "ResourceManager.h"
#include "PhysicsManager.h"
#include "SceneGraph.h"
#include "DestructionEngine.h"
#include "VoxelConverter.h"
#include "console.h"

bool App::DEBUG_PHYSICS = false;
bool App::DEBUG_INTERAL_SIMULATION = false;
bool App::DEBUG_MARCHING_CUBES = false;
bool App::DEBUG_VOX_DECOMP = false;
bool App::DEBUG_BREAKING = false;
bool App::DEBUG_BRIDGES = false;

App::App()
{
    mVoxelConversionCPU = false;
}

App::~App()
{
    delete mRenderer;
    delete mResourceManager;
    delete mSceneGraph; //Must delete before physics manager.
    delete mPhysicsManager;
    delete mVoxelConverter;
    delete mDestructionEngine;
}

void App::init()
{
    mLastTickTime = glfwGetTime();
    mLastFrameTime = mLastTickTime;
    mLastSimulationTime = glfwGetTime();

     mRenderer = new Renderer();
    mResourceManager = new ResourceManager();
    mPhysicsManager = new PhysicsManager(this);
    mSceneGraph = new SceneGraph();
    mVoxelConverter = new VoxelConverter(this);
    mDestructionEngine = new DestructionEngine(this);

    if(mVoxelConversionCPU)
    {
        mVoxelConverter->initCPU();
    }
    else
    {
        if(!mVoxelConverter->initGPU("shader/list_triangles.vert", "shader/list_triangles.geom",
                            "shader/gen_vertices.vert", "shader/gen_vertices.geom"))
        {
            mVoxelConverter->initCPU();
        }
    }

    mFrameCount = 0;
    mTickCount = 0;
}

Renderer* App::getRenderer()
{
    return mRenderer;
}

ResourceManager* App::getResourceManager()
{
    return mResourceManager;
}

PhysicsManager* App::getPhysicsManager()
{
    return mPhysicsManager;
}

SceneGraph* App::getSceneGraph()
{
    return mSceneGraph;
}

DestructionEngine* App::getDestructionEngine()
{
    return mDestructionEngine;
}

VoxelConverter* App::getVoxelConverter()
{
    return mVoxelConverter;
}

void App::setFrameFrequency(GLdouble frameFreq)
{
    mFrameFreq = frameFreq;
}

void App::setTickFrequency(GLdouble tickFreq)
{
    mTickFreq = tickFreq;
}

GLboolean App::timeToRender()
{
    if(mFrameCount < 1)
    {
        return true;
    }
    GLdouble currentTime = glfwGetTime();
    GLboolean render = (currentTime - mLastFrameTime) > mFrameFreq;
    if(render)
    {
        return true;
    }
    else
    {
        return false;
    }
}

GLboolean App::timeToSimulate()
{
    if(mTickCount < 1)
    {
        return true;
    }
    GLdouble currentTime = glfwGetTime();
    return (currentTime - mLastTickTime > mTickFreq);
}

GLdouble App::sleepTime()
{
    if(mTickCount < 1 || mFrameCount < 1)
    {
        return 0;
    }

    GLdouble currentTime = glfwGetTime();
    GLdouble timeToNextFrame = (mFrameFreq + mLastFrameTime) - currentTime;
    GLdouble timeToNextSim = (mTickFreq + mLastTickTime) - currentTime;

    if(timeToNextFrame < timeToNextSim)
    {
        return timeToNextFrame;
    }
    else
    {
        return timeToNextSim;
    }
}

GLdouble App::deltaTime()
{
    if(mTickCount < 1)
    {
        return 0;
    }
    GLdouble currentTime = glfwGetTime();
    return currentTime - mLastSimulationTime;
}

void App::useCPUforVoxelConversion(GLboolean setting)
{
    mVoxelConversionCPU = setting;
}

void App::clockTick()
{
    mLastTickTime +=  mTickFreq;
    mLastSimulationTime = glfwGetTime();
    mTickCount++;
}

void App::clockFrame()
{
    mLastFrameTime += mFrameFreq;
    mFrameCount++;
}

void App::debugPrint(bool notMuted, const std::string& message)
{
    if(notMuted) std::cout << "D: " << message << std::endl;
}

void App::debugPrint(bool notMuted, const std::string& message, int value)
{
    if(notMuted) std::cout << "D: " << message << value << std::endl;
}

void App::debugPrint(bool notMuted, const std::string& message, float value)
{
    if(notMuted) std::cout << "D: " << message << value << std::endl;
}

void App::debugPrint(bool notMuted, const std::string& message, double value)
{
    if(notMuted) std::cout << "D: " << message << value << std::endl;
}
