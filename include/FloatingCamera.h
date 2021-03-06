#ifndef FLOATINGCAMERA_H
#define FLOATINGCAMERA_H

#include <CameraNode.h>

const double RECHARGE_TIME = 1.0;
const float BULLET_FORCE = -50000.0f;

/*
    Class: FloatingCamera
    Subclass of CameraNode that moves due to the user's input and can shoot bullets.
*/
class FloatingCamera : public CameraNode
{
    public:
        FloatingCamera(App* app, GLenum renderPass, GLdouble fov, GLdouble zNear, GLdouble zFar);
        virtual ~FloatingCamera();

        void simulateSelf(GLdouble deltaTime);
    protected:

        double mRotX;
        double mRotY;

        double mRechargeTime;
    private:
};

#endif // FLOATINGCAMERA_H
