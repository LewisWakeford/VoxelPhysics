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
}

int main()
{
    try{
        int     width, height;
        bool    running = true;
        std::string optionChoice = "";

        consolePrint("Initialising GLFW");

        if(GL_TRUE != glfwInit())
        {
            consolePrint("ERROR: Initialisation Failed.");
            return 0;
        }
        errorCheck(25);

        consolePrint("Creating Window");

        if( !glfwOpenWindow(1920, 1080, 0, 0, 0, 0, 0, 0, GLFW_FULLSCREEN ) )
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
        errorCheck(46);

        if(!GLEW_ARB_transform_feedback2 && !GLEW_ARB_transform_feedback3)
        {
            consolePrint("Transform feedback is unsupported, reverting to CPU algorithm.");
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
        theApp.setFrameFrequency(1.0/60.0);
        theApp.setTickFrequency(1.0/120.0);
        VoxelConverter* voxConv = theApp.getVoxelConverter();
        PhysicsManager* physicsManager = theApp.getPhysicsManager();
        Renderer* renderer = theApp.getRenderer();
        SceneGraph* sceneGraph = theApp.getSceneGraph();
        sceneGraph->setRoot(SceneNodePtr(new SceneNode(&theApp, VP_RENDER_NONE)));

        renderer->setDimensions((GLdouble)width, (GLdouble)height);

        SceneNodePtr cameraPtr(new FloatingCamera(&theApp, VP_RENDER_GEOMETRY, 90.0f, 1.0f, 1000.0f));
        ((FloatingCamera*)cameraPtr.get())->pitch(1.5f);
        ((FloatingCamera*)cameraPtr.get())->yaw(3.2f);
        ((FloatingCamera*)cameraPtr.get())->fly(-20.0f);
        ((FloatingCamera*)cameraPtr.get())->walk(-30.0f);

        renderer->setCamera((CameraNode*)cameraPtr.get());

        sceneGraph->getRoot()->addChild(cameraPtr);

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

        Material* testMaterial = new Material();

        {
            SceneNodePtr matterNode1(new MatterNode(&theApp, VP_RENDER_GEOMETRY, testMaterial, false, "vox/house.vox"));
            ((MatterNode*)matterNode1.get())->setOffset(0.0f, 0.0f, 20.0f);
            sceneGraph->getRoot()->addChild(matterNode1);

            SceneNodePtr matterNode2(new MatterNode(&theApp, VP_RENDER_GEOMETRY, testMaterial, false, "vox/test_big_cube.vox"));
            ((MatterNode*)matterNode2.get())->setOffset(15.0f, 15.0f, 200.0f);
            //sceneGraph->getRoot()->addChild(matterNode2);
        }


        //SceneNodePtr matterNode3(new MatterNode(&theApp, VP_RENDER_GEOMETRY, &blue[0], false, "vox/atom.vox"));
        //((MatterNode*)matterNode3.get())->setOffset(0.0f, 0.0f, 60.0f);
        //sceneGraph->getRoot()->addChild(matterNode3);

       // worldLightProgram.use();

        GLint worldLightColor = glGetUniformLocation(worldLightProgram.getID(), "worldLightColor");
        glUniform4f(worldLightColor, 0.8f, 0.8f, 0.8f, 1.0f);

        GLint worldLightVector = glGetUniformLocation(worldLightProgram.getID(), "worldLightVector");
        glUniform3f(worldLightVector, 0.0f, 0.0f, -1.0f);

        GLint ambientColor = glGetUniformLocation(worldLightProgram.getID(), "ambientColor");
        glUniform4f(ambientColor, 0.3f, 0.3f, 0.3f, 1.0f);

        unsigned int frames = 0;
        double lastFPSUpdate = 0.0;
        double elapsedTime = 0.0;

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        while(running)
        {
            GLdouble timeToWait = theApp.sleepTime();
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
/*
            worldLightProgram.use();

            GLint worldLightColor = glGetUniformLocation(worldLightProgram.getID(), "worldLightColor");
            glUniform4f(worldLightColor, 0.8f, 0.8f, 0.8f, 1.0f);

            GLint worldLightVector = glGetUniformLocation(worldLightProgram.getID(), "worldLightVector");
            glUniform3f(worldLightVector, 0.0f, 0.0f, 1.0f);

            GLint ambientColor = glGetUniformLocation(worldLightProgram.getID(), "ambientColor");
            glUniform4f(ambientColor, 0.3f, 0.3f, 0.3f, 1.0f);
*/
            //Rendering code
           //if(theApp.timeToRender())
            {
                //theApp.clockFrame();

                sceneGraph->onRender(VP_RENDER_GEOMETRY);
                renderer->render();
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
        delete testMaterial;
    }
    catch (std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }

    glfwTerminate();

    return 0;

}
