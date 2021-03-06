#include "VP.h"

#include <iostream>
#include <exception>

#include "console.h"
#include "Shader.h"
#include "App.h"
#include "VoxelField.h"
#include "VertexShell.h"
#include "ShaderProgram.h"
#include "VoxelConverter.h"
#include "PhysicsManager.h"
#include "Renderer.h"
#include "FloatingCamera.h"
#include "SceneNode.h"
#include "SceneGraph.h"
#include "MatterNode.h"

App theApp;

void buildScene()
{
    //Delete old scene

    theApp.getSceneGraph()->setRoot(SceneNodePtr(new SceneNode(&theApp, VP_RENDER_NONE)));

    SceneNodePtr matterNode1(new MatterNode(&theApp, VP_RENDER_GEOMETRY, theApp.gDefaultMaterial, false, "vox/test_small_cube.vox"));
    ((MatterNode*)matterNode1.get())->setOffset(0.0f, 0.0f, 20.0f);
    theApp.getSceneGraph()->getRoot()->addChild(matterNode1);

    SceneNodePtr matterNode2(new MatterNode(&theApp, VP_RENDER_GEOMETRY, theApp.gDefaultMaterial, false, "vox/building.vox"));
    ((MatterNode*)matterNode2.get())->setOffset(-50.0f, -50.0f, 20.0f);
    theApp.getSceneGraph()->getRoot()->addChild(matterNode2);

    SceneNodePtr matterNode3(new MatterNode(&theApp, VP_RENDER_GEOMETRY, theApp.gDefaultMaterial, false, "vox/ledge.vox"));
    ((MatterNode*)matterNode3.get())->setOffset(50.0f, 50.0f, 20.0f);
    theApp.getSceneGraph()->getRoot()->addChild(matterNode3);

    SceneNodePtr cameraPtr(new FloatingCamera(&theApp, VP_RENDER_GEOMETRY, 90.0f, 1.0f, 1000.0f));
    ((FloatingCamera*)cameraPtr.get())->pitch(1.5f);
    ((FloatingCamera*)cameraPtr.get())->yaw(3.2f);
    ((FloatingCamera*)cameraPtr.get())->fly(-20.0f);
    ((FloatingCamera*)cameraPtr.get())->walk(-30.0f);

    theApp.getRenderer()->setCamera((CameraNode*)cameraPtr.get());

    theApp.getSceneGraph()->getRoot()->addChild(cameraPtr);

    //SceneNodePtr matterNode2(new MatterNode(&theApp, VP_RENDER_GEOMETRY, theApp.gDefaultMaterial, false, "vox/donut.vox"));
    //((MatterNode*)matterNode2.get())->setOffset(15.0f, 15.0f, 200.0f);
    //sceneGraph->getRoot()->addChild(matterNode2);

}

void useBullets()
{
    theApp.gBulletFile = "vox/atom.vox";
    theApp.gBulletForce = -50000.0f;
}

void useCubes()
{
    theApp.gBulletFile = "vox/test_small_cube.vox";
    theApp.gBulletForce = -5000000.0f;
}

void GLFWCALL keyCallback(int key, int action)
{
     //A - Move Left
    if(key == 65)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving left.
            theApp.gMoveLeft = true;
        }
        else
        {
            //Player stop moving left.
            theApp.gMoveLeft = false;
        }
    }

    //D - Move Right
    if(key == 68)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gMoveRight = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gMoveRight = false;
        }
    }

    //W - Move forward
    if(key == 87)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gMoveForward = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gMoveForward = false;
        }
    }

    //S - Move backward
    if(key == 83)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gMoveBackward = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gMoveBackward = false;
        }
    }

    //Up - Pitch up
    if(key == GLFW_KEY_UP)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gPitchUp = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gPitchUp = false;
        }
    }

    //Down - Pitch down
    if(key == GLFW_KEY_DOWN)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gPitchDown = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gPitchDown = false;
        }
    }

    //Left - Yaw Left
    if(key == GLFW_KEY_LEFT)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gYawLeft = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gYawLeft = false;
        }
    }

    //Right - Yaw Right
    if(key == GLFW_KEY_RIGHT)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving right.
            theApp.gYawRight = true;
        }
        else
        {
            //Player stop moving right.
            theApp.gYawRight = false;
        }
    }

    //Space - Move up
    if(key == GLFW_KEY_SPACE)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving up
            theApp.gMoveUp = true;
        }
        else
        {
            //Player stop moving up
            theApp.gMoveUp = false;
        }
    }

    //Ctrl - Move down
    if(key == GLFW_KEY_LCTRL)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving down
            theApp.gMoveDown = true;
        }
        else
        {
            //Player stop moving down
            theApp.gMoveDown = false;
        }
    }

    //Q - Roll Left (counter clockwise)
    if(key == 81)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving down
            theApp.gRollLeft = true;
        }
        else
        {
            //Player stop moving down
            theApp.gRollLeft = false;
        }
    }

    //E - Roll Right (clockwise)
    if(key == 69)
    {
        if(action == GLFW_PRESS)
        {
            //Player start moving down
            theApp.gRollRight = true;
        }
        else
        {
            //Player stop moving down
            theApp.gRollRight = false;
        }
    }

    //R - Shoot ball.
    if(key == 82)
    {
        if(action == GLFW_PRESS)
        {
            theApp.gShoot = true;
        }
        else
        {
            theApp.gShoot = false;
        }
    }

    //1 - use balls
    if(key == 49)
    {
        if(action == GLFW_RELEASE)
        {
            useBullets();
        }
    }

    //2 - use cubes -- Removed due to bug.
   /* if(key == 50)
    {
        if(action == GLFW_RELEASE)
        {
            useCubes();
        }
    }
*/
    //P - reset
    if(key == 80)
    {
        if(action == GLFW_RELEASE)
        {
            buildScene();
        }
    }
    if(key == 84)
    {
        if(action == GLFW_RELEASE)
        {
            theApp.gPhysics = !theApp.gPhysics;
        }
    }
}

int main(int numArgs, char* args[])
{
    try{
        int     width, height;
        bool    running = true;
        bool forceCPU = false;

        theApp.gPhysics = true;
        theApp.gDestructionEnabled = true;
        useBullets();

        for(int i = 0; i < numArgs; i++)
        {
            std::string arg(args[i]);
            std::cout << arg << std::endl;
            if(arg == "-forceCPU") forceCPU = true;
        }

        consolePrint("Initialising GLFW");

        if(GL_TRUE != glfwInit())
        {
            consolePrint("ERROR: Initialisation Failed.");
            return 0;
        }
        errorCheck(__LINE__, __FILE__);

        consolePrint("Creating Window");

        if( !glfwOpenWindow(1920, 1080, 0, 0, 0, 0, 0, 0, GLFW_WINDOW ) )
        {
            consolePrint("ERROR: Window Creation Failed.");
            glfwTerminate();
            return 0;
        }
        glfwGetWindowSize(&width, &height);

        consolePrint("Initialising GLEW");

        GLenum err = glewInit();
        if(GLEW_OK != err)
        {
            consolePrint("GLEW Failed to initalise.");
            consolePrint((char*)glewGetErrorString(err));
            return 0;
        }
        errorCheck(__LINE__, __FILE__);

        if(!forceCPU)
        {
            if(!GLEW_ARB_transform_feedback2 && !GLEW_ARB_transform_feedback3)
            {
                consolePrint("Transform feedback is unsupported, reverting to CPU algorithm.");
                theApp.useCPUforVoxelConversion(true);
            }
        }
        else
        {
            consolePrint("Forcing CPU mode");
            theApp.useCPUforVoxelConversion(true);
        }


        //Render Setup
        Shader vertWorldLight(GL_VERTEX_SHADER_ARB);
        vertWorldLight.setSource("shader/world_light.vert");
        vertWorldLight.compile();
        vertWorldLight.errorCheck();

        Shader fragWorldLight(GL_FRAGMENT_SHADER_ARB);
        fragWorldLight.setSource("shader/world_light.frag");
        fragWorldLight.compile();
        fragWorldLight.errorCheck();

        ShaderProgram worldLightProgram;
        worldLightProgram.attach(vertWorldLight);
        worldLightProgram.attach(fragWorldLight);
        worldLightProgram.link();

        //App setup

        glfwSetKeyCallback(keyCallback);

        theApp.init();
        VoxelConverter* voxConv = theApp.getVoxelConverter();
        PhysicsManager* physicsManager = theApp.getPhysicsManager();
        Renderer* renderer = theApp.getRenderer();
        SceneGraph* sceneGraph = theApp.getSceneGraph();
        sceneGraph->setRoot(SceneNodePtr(new SceneNode(&theApp, VP_RENDER_NONE)));

        renderer->setDimensions((GLdouble)width, (GLdouble)height);

        //End app setup

        glEnable(GL_TEXTURE_3D);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_CULL_FACE);
        glEnable(GL_PROGRAM_POINT_SIZE);
        glPointSize(5.0f);

        glfwSetWindowTitle("Voxel Physics");
        glClearColor(0.3f, 0.3f, 0.3f, 1.0f);

        //Create matter nodes
        GLfloat red[3] = {1.0f, 0.0f, 0.0f};
        GLfloat green[3] = {0.0f, 1.0f, 0.0f};
        GLfloat blue[3] = {0.0f, 0.0f, 1.0f};

        theApp.gDefaultMaterial = MaterialPtr(new Material());
        theApp.gBulletMaterial = MaterialPtr(new Material());
        theApp.gBulletMaterial->setDensity(20.0f);
        theApp.gBulletMaterial->setStrength(0.9, 1.1);
        theApp.gBulletMaterial->setPressureLimit(100000000000.0f);
        theApp.gBulletMaterial->setStressLimit(100000000000.0f);
        theApp.gBulletMaterial->setBondStrength(10.0f);
        theApp.gBulletMaterial->setColor(0.0f, 0.0f, 0.0f);

        buildScene();

        unsigned int frames = 0;
        double lastFPSUpdate = 0.0;
        double elapsedTime = 0.0;

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        std::cout << "System Initialised... type something to continue: ";
        std::string cmd;
        std::cin >> cmd;

        while(running)
        {
            GLdouble deltaTime = theApp.deltaTime();

           //consolePrint("", deltaTime);

            //Simulation code
           //if(theApp.timeToSimulate())
            {

                physicsManager->simulate(deltaTime);
                sceneGraph->onLoop(deltaTime);
                theApp.clockTick();
            }

            glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            //Rendering code
           //if(theApp.timeToRender())
            {
                //theApp.clockFrame();

                sceneGraph->onRender(VP_RENDER_GEOMETRY);
                glfwSwapBuffers();

                theApp.clockFrame();
                frames++;
            }

            elapsedTime = glfwGetTime();

            if(elapsedTime-lastFPSUpdate >= 1.0)
            {
                double fps = frames/(elapsedTime-lastFPSUpdate);
                lastFPSUpdate = elapsedTime;
                frames = 0;
                consolePrint("FPS: ", fps);
            }

            // exit if ESC was pressed or window was closed
            running = !glfwGetKey(GLFW_KEY_ESC) && glfwGetWindowParam( GLFW_OPENED);
        }
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
        std::cout << "Type something to end program";
        std::string cmd;
        std::cin >> cmd;
    }

    glfwTerminate();

    return 0;

}
