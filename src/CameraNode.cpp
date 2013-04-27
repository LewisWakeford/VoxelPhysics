#include "CameraNode.h"
#include "App.h"
#include "Renderer.h"

CameraNode::CameraNode(App* app, GLenum renderPass, GLdouble fov, GLdouble zNear, GLdouble zFar) :
    SceneNode(app, renderPass)
{
    mFOV = fov;
    mZNear = zNear;
    mZFar = zFar;

    mPitch = 0;
    mYaw = 0;
    mRoll = 0;

    mPos.set(0,0,0);
    mForward.set(0,0,1);
    mUp.set(0,1,0);
    mRight.set(1,0,0);

    mCanRoll = true;
}

CameraNode::~CameraNode()
{
    //dtor
}

void CameraNode::renderSelf()
{
    mApp->getRenderer()->updateCamera(this);
}

GLdouble CameraNode::getFOV()
{
    return mFOV;
}

GLdouble CameraNode::getZNear()
{
    return mZNear;
}

GLdouble CameraNode::getZFar()
{
    return mZFar;
}

GLfloat CameraNode::getPitch()
{
    return mPitch;
}

GLfloat CameraNode::getYaw()
{
    return mYaw;
}

GLfloat CameraNode::getRoll()
{
    return mRoll;
}

Vector3f CameraNode::getPos()
{
    return mPos;
}

void CameraNode::yaw(GLfloat angle)
{
    Matrix4D mat = Matrix4D::createArbRotation(mUp, angle);
    mRight = mat.transformVertex(mRight);
    mForward = mat.transformVertex(mForward);

    mRight.normalize();
    mForward.normalize();
}

void CameraNode::pitch(GLfloat angle)
{
    Matrix4D mat = Matrix4D::createArbRotation(mRight, angle);
    mUp = mat.transformVertex(mUp);
    mForward = mat.transformVertex(mForward);

    mUp.normalize();
    mForward.normalize();
}

void CameraNode::roll(GLfloat angle)
{
    Matrix4D mat = Matrix4D::createArbRotation(mForward, angle);

    mRight = mat.transformVertex(mRight);
    mUp = mat.transformVertex(mUp);

    mRight.normalize();
    mUp.normalize();
}

void CameraNode::fly(GLfloat amount)
{
    mPos.x += mUp.x * amount;
    mPos.y += mUp.y * amount;
    mPos.z += mUp.z * amount;
}

void CameraNode::strafe(GLfloat amount)
{
    mPos.x += mRight.x * amount;
    mPos.y += mRight.y * amount;
    mPos.z += mRight.z * amount;
}

void CameraNode::walk(GLfloat amount)
{
    mPos.x += mForward.x * amount;
    mPos.y += mForward.y * amount;
    mPos.z += mForward.z * amount;
}

Matrix4D CameraNode::getCameraMatrix()
{
    Matrix4D camMat;

    camMat.setRow(0, mRight.x,      mRight.y,   mRight.z,   0);
    camMat.setRow(1, mUp.x,         mUp.y,      mUp.z,      0);
    camMat.setRow(2, mForward.x,    mForward.y, mForward.z, 0);
    camMat.setRow(3, 0,             0,          0,          1);

    Matrix4D transMat = Matrix4D::createTranslation(mPos.x, mPos.y, mPos.z);

    camMat.multiplyBy(&transMat);

    /*
    camMat.setRow(0, mRight.x,  mUp.x,  mForward.x, mPos.x);
    camMat.setRow(1, mRight.y,  mUp.y,  mForward.y, mPos.y);
    camMat.setRow(2, mRight.z,  mUp.z,  mForward.z, mPos.z);
    camMat.setRow(3, 0,             0,          0,          1);
    */

    return camMat;
}
