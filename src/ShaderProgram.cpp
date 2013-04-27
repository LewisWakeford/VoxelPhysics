#include "ShaderProgram.h"

#include "Shader.h"
#include "console.h"

ShaderProgram::ShaderProgram()
{
   mID = glCreateProgramObjectARB();
}

ShaderProgram::~ShaderProgram()
{
    //dtor
}

void ShaderProgram::attach(Shader shader)
{
    glAttachObjectARB(mID, shader.getID());
}

void ShaderProgram::link()
{
    glLinkProgramARB(mID);
}

GLboolean ShaderProgram::checkLink()
{
    GLint linkStatus;
    glGetProgramiv(mID, GL_LINK_STATUS, &linkStatus);
    GLsizei length;
    GLchar message[200];
    glGetProgramInfoLog(mID, 200, &length, message);
    consolePrint(message);

    if(!linkStatus)
    {
        consolePrint("Shader failed to link: " + mID);
        return false;
    }
    else
    {
        return true;
    }
}

void ShaderProgram::use()
{
    glUseProgramObjectARB(mID);
}

GLenum ShaderProgram::getID()
{
    return mID;
}
