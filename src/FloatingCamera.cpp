#include "FloatingCamera.h"
#include "App.h"
#include <cmath>
#include "console.h"

FloatingCamera::FloatingCamera(App* app, GLenum renderPass, GLdouble fov, GLdouble zNear, GLdouble zFar)
    : CameraNode(app, renderPass, fov, zNear, zFar)
{
    mRotX = 0.0;
    mRotY = 0.0;

    mCanRoll = false;
}

FloatingCamera::~FloatingCamera()
{
    //dtor
}

void FloatingCamera::simulateSelf(GLdouble deltaTime)
{

    double pyrSpd = -5;
    double pyrRot = 3;
    Matrix4D newTransform = Matrix4D::createIdentity();

    //Create movement vector
    Vector3 dVec = {0,0,0};
    double dYaw = 0.0;
    double dPitch = 0.0;
    double dRoll = 0.0;

    if(mApp->gMoveForward)
    {
        dVec.z = -pyrSpd*deltaTime;
    }
    else if(mApp->gMoveBackward)
    {
        dVec.z = pyrSpd*deltaTime;
    }

    if(mApp->gMoveLeft)
    {
        dVec.x = -pyrSpd*deltaTime;
    }
    else if(mApp->gMoveRight)
    {
        dVec.x = pyrSpd*deltaTime;
    }

    if(mApp->gMoveUp)
    {
        dVec.y = pyrSpd*deltaTime;
    }
    else if(mApp->gMoveDown)
    {
        dVec.y = -pyrSpd*deltaTime;
    }

   if(mApp->gYawLeft)
    {
        dYaw = pyrRot*deltaTime;
    }
    else if(mApp->gYawRight)
    {
        dYaw = -pyrRot*deltaTime;
    }

    if(mApp->gPitchUp)
    {
        dPitch = pyrRot*deltaTime;
    }
    else if(mApp->gPitchDown)
    {
        dPitch = -pyrRot*deltaTime;
    }

    if(mApp->gRollLeft)
    {
        dRoll = pyrRot*deltaTime;
    }
    else if(mApp->gRollRight)
    {
        dRoll = -pyrRot*deltaTime;
    }

    yaw(dYaw);
    pitch(dPitch);
    roll(dRoll);

    walk(dVec.z);
    fly(dVec.y);
    strafe(dVec.x);
}
