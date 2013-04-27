#ifndef CAMERANODE_H
#define CAMERANODE_H

#include "SceneNode.h"
#include "Vector3.h"

class CameraNode : public SceneNode
{
    public:
        CameraNode(App* app, GLenum renderPass, GLdouble fov, GLdouble zNear, GLdouble zFar);
        virtual ~CameraNode();

        void renderSelf();

        GLdouble getFOV();
        GLdouble getZNear();
        GLdouble getZFar();

        void pitch(GLfloat angle);
        void yaw(GLfloat angle);
        void roll(GLfloat angle);

        void fly(GLfloat amount);
        void strafe(GLfloat amount);
        void walk(GLfloat amount);

        GLfloat getPitch();
        GLfloat getYaw();
        GLfloat getRoll();

        Vector3f getPos();
        Vector3f getUp();
        Vector3f getRight();
        Vector3f getForward();

        Matrix4D getCameraMatrix();

    protected:
        GLdouble mFOV;
        GLdouble mZNear;
        GLdouble mZFar;

        GLfloat mPitch;
        GLfloat mYaw;
        GLfloat mRoll;

        Vector3f mPos;
        Vector3f mForward;
        Vector3f mUp;
        Vector3f mRight;

        GLboolean mCanRoll;

    private:
};

#endif // CAMERANODE_H
