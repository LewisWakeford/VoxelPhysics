#ifndef CAMERANODE_H
#define CAMERANODE_H

#include "SceneNode.h"
#include "Vector3.h"

/*
    Class: CameraNode
    Helps position the camera.
*/

class CameraNode : public SceneNode
{
    public:
        CameraNode(App* app, GLenum renderPass, GLdouble fov, GLdouble zNear, GLdouble zFar);
        virtual ~CameraNode();

        //Update the renderer with the this camera's posistion. If this is the current camera, that is.
        void renderSelf();

        //Return camera settings
        GLdouble getFOV();
        GLdouble getZNear();
        GLdouble getZFar();

        //Pitch, Yaw or Roll Camera around local axes.
        void pitch(GLfloat angle);
        void yaw(GLfloat angle);
        void roll(GLfloat angle);

        //Move the camera along the local axes.
        void fly(GLfloat amount); //Y axis
        void strafe(GLfloat amount); //Z axis
        void walk(GLfloat amount); //X axis

        GLfloat getPitch();
        GLfloat getYaw();
        GLfloat getRoll();

        Vector3f getPos();
        Vector3f getUp();
        Vector3f getRight();
        Vector3f getForward();

        //Get the projection matrix
        Matrix4D getCameraMatrix();

    protected:

        GLdouble mFOV; //Field of View
        GLdouble mZNear; //Near Clipping Plane
        GLdouble mZFar; //Far Clipping Plane

        Vector3f mPos;
        Vector3f mForward;
        Vector3f mUp;
        Vector3f mRight;

        GLboolean mCanRoll;

    private:
};

#endif // CAMERANODE_H
