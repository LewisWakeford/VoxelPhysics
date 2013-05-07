#ifndef FLOATINGCAMERA_H
#define FLOATINGCAMERA_H

#include <CameraNode.h>

const double RECHARGE_TIME = 1.0;
const float BULLET_FORCE = 1000;

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
