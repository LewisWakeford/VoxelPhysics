#include "Renderer.h"
#include "Matrix4D.h"
#include "Mesh.h"
#include "CameraNode.h"
#include "VertexShell.h"
#include "Buffer.h"
#include "Matter.h"

Renderer::Renderer()
{
    mCameraMatrix = Matrix4D::createIdentity();
}

Renderer::~Renderer()
{
    //dtor
}

void Renderer::pushMatrix(Matrix4D matrix)
{
    if(mMatrixStack.size() > 0)
    {
        Matrix4D temp = mMatrixStack.back();
        temp.multiplyBy(&matrix);
        mMatrixStack.push_back(temp);
    }
    else
    {
        Matrix4D temp = matrix;
        mMatrixStack.push_back(temp);
    }
}

Matrix4D Renderer::popMatrix()
{
    Matrix4D temp = mMatrixStack.back();
    mMatrixStack.pop_back();
    return temp;
}

Matrix4D Renderer::currentMatrix()
{
    Matrix4D temp = mMatrixStack.back();
    return temp;
}

/* Rip from SI4D when needed
GLboolean Renderer::renderMesh(Mesh* mesh)
{
    //Cannot render a mesh without vertices.
    if(mesh->hasVertices() != GL_TRUE)
    {
        return GL_FALSE;
    }

    //Retrieve the vertex data from the mesh.
    std::vector<GLushort> meshFaces = mesh->getFaces();
    std::vector<GLfloat> meshVertices = mesh->getVertices();

    //Faces must be offset because they refer to indices in the vertex arrays.
    GLint offset = (mVertices.size()/3);
    if(offset < 0) offset = 0;

    //Concat the face vectors:
    for(unsigned int i = 0; i < meshFaces.size(); i++)
    {
        mFaces.push_back((meshFaces[i]+offset)-1);
    }


    //Concat the vertex vectors, after transforming them by the current matrix.

    for(unsigned int i = 0; i < meshVertices.size(); i+=3)
    {
        //TRANSFORM VERTEX VIA CURRENT MATRIX
        Vertex3 temp = {meshVertices[i],meshVertices[i+1],meshVertices[i+2]};

        if(mMatrixStack.size() > 0)
        {
            temp = currentMatrix().transformVertex(temp);
        }
        mVertices.push_back(temp.x);
        mVertices.push_back(temp.y);
        mVertices.push_back(temp.z);
    }

    if(mesh->hasNormals() != GL_TRUE)
    {
        //Would maybe generate normals here or tell GL to do it for us.
        //For now we can leave it as we are just using flat colors.
    }

    if(mesh->hasTextures() && mesh->getType() & VP_MESH_TEXTURED)
    {
        //Get Textures
        //Setup for Textured Rendering
    }
    else if(mesh->hasColors() && mesh->getType() & VP_MESH_COLORED)
    {
        //Get Colors
        std::vector<GLfloat> meshColors = mesh->getColors();

        for(unsigned int i = 0; i < meshVertices.size()/3; i++)
        {
            mColors.push_back(meshColors[0]);
            mColors.push_back(meshColors[1]);
            mColors.push_back(meshColors[2]);
            mColors.push_back(meshColors[3]);
        }

        //Setup for Colored Rendering
    }
    else
    {
        return GL_FALSE;
    }

    return GL_TRUE;
}
*/

GLboolean Renderer::renderMatter(Matter& matter)
{
    //Draw immediatly.
    glDisable(GL_TEXTURE_3D);

    //Camera setup
    if(mCamera)
    {
        GLfloat* values = mCameraMatrix.getValues();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glMultMatrixf(values);
    }

    if(matter.hasCollided())
    {
        glColor3fv(matter.getColorPtr());
    }
    else
    {
        glColor3f(1.0f, 1.0f, 1.0f);
    }
    //
    //glColor3f(1.0f, 1.0f, 1.0f);
    glMultMatrixf(currentMatrix().getValues());

    matter.getVertexShell()->getBuffer()->render();
    //matter.debugRenderVoxels();
    //matter.debugRenderHulls();
    //matter.debugRenderPressure();
}

void Renderer::render()
{
    //Camera setup
    if(mCamera)
    {
        GLfloat* values = mCameraMatrix.getValues();

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glMultMatrixf(values);
    }

    glDisable(GL_TEXTURE_3D);
    glColor3f(0.0f, 1.0f, 0.0f);

}

void Renderer::setCamera(CameraNode* camera)
{
    mCamera = camera;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(camera->getFOV(), mViewportWidth/mViewportHeight, camera->getZNear(), camera->getZFar());
}

void Renderer::updateCamera(CameraNode* camera)
{
    if(camera == mCamera)
    {
       mCameraMatrix = camera->getCameraMatrix();
       //mCameraMatrix.invert(); Need this??
    }

}

void Renderer::setDimensions(GLdouble width, GLdouble height)
{
    mViewportWidth = width;
    mViewportHeight = height;
}
