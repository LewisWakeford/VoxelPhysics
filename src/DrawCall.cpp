#include "DrawCall.h"

#include "App.h"
#include "ResourceManager.h"
#include "Material.h"

DrawCall::DrawCall(App* app, GLint materialID)
{

}

DrawCall::DrawCall(App* app, GLint materialID, GLuint bufferName)
{

}

DrawCall::~DrawCall()
{
    //dtor
}

void DrawCall::setVertexBuffer(GLuint bufferName, GLuint faceCount)
{

}

void DrawCall::setMatrixBuffer(GLuint bufferName)
{

}

GLboolean DrawCall::draw()
{
    /*
    //Use the material for this draw call
    mApp->getResourceManager()->getMaterial(MaterialID)->use();

    //Bind vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, mVertexBufferName);

    //Point at properties
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_NORMAL_ARRAY);

    glVertexPointer(3, GL_FLOAT, sizeof(GLfloat)*3, 0);
    glNormalPointer(GL_FLOAT, sizeof(GLfloat)*3, sizeof(GLfloat)*3);

    glDrawArrays(GL_TRIANGLES, 0, mFaceCount);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);

    */

    return true;
}
