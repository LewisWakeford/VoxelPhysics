#ifndef RENDERER_H
#define RENDERER_H

#include "VP.h"

#include <vector>

#include "Matrix4D.h"

class Mesh;
class Billboard;
class CameraNode;
class VertexShell;
class Buffer;
class Matter;
class App;

class Renderer
{
    public:
        Renderer(App* app);
        virtual ~Renderer();

        GLboolean renderMesh(Mesh* mesh);
        GLboolean renderMatter(Matter& matter); //Render a matter chunk.
        GLboolean renderBillboard(Billboard* billboard);

        void render();

        void pushMatrix(Matrix4D matrix);
        Matrix4D popMatrix();
        Matrix4D currentMatrix();

        void setCamera(CameraNode* camera); //Set the current camera
        void updateCamera(CameraNode* camera); //All cameras call this during their simulation, if the camera is the current camera then the camera matrix is updated.

        void setDimensions(GLdouble width, GLdouble height);


    protected:

        std::vector<Matrix4D> mMatrixStack;

        //std::vector<DrawCall> mDrawCalls; //Currently not used, but essentially each draw call will contain all (or most) of the vertex data for a particular material
        std::vector<Buffer*> mBuffers; //May remove later. Render each buffer one at a time.

        CameraNode* mCamera;
        GLfloat mCameraPitch;
        GLfloat mCameraYaw;
        GLfloat mCameraRoll;
        Vector3 mCameraPos;
        Matrix4D mCameraMatrix; //Inverse of mCamera's tranform.

        GLdouble mViewportWidth;
        GLdouble mViewportHeight;

        App* mApp;

    private:
};

#endif // RENDERER_H
