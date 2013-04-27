#ifndef FLOATINGCAMERA_H
#define FLOATINGCAMERA_H

#include <CameraNode.h>


class FloatingCamera : public CameraNode
{
    public:
        FloatingCamera(App* app, GLenum renderPass, GLdouble fov, GLdouble zNear, GLdouble zFar);
        virtual ~FloatingCamera();

        void simulateSelf(GLdouble deltaTime);
    protected:

        double mRotX;
        double mRotY;
    private:
};

#endif // FLOATINGCAMERA_H
