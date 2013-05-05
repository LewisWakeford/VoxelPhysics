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
    glAttachShader(mID, shader.getID());
}

void ShaderProgram::link()
{
    glLinkProgram(mID);
}

GLboolean ShaderProgram::checkLink()
{
    GLint linkStatus;
    GLint maxLength;
    glGetProgramiv(mID, GL_LINK_STATUS, &linkStatus);
    glGetProgramiv(mID, GL_INFO_LOG_LENGTH, &maxLength);

    if(maxLength > 0)
    {
        GLsizei length;
        char* message = new char[maxLength];
        glGetProgramInfoLog(mID, maxLength, &length, message);
        std::string infoString(message, length);

        std::cout << "Shader info log: " << infoString << std::endl;

        delete[] message;
    }

    if(!linkStatus)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void ShaderProgram::use()
{
    glUseProgram(mID);
}

GLenum ShaderProgram::getID()
{
    return mID;
}
