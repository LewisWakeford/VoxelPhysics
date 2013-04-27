#include "Shader.h"
#include "console.h"

Shader::Shader(GLenum shaderType)
{
    mID = glCreateShaderObjectARB(shaderType);
}

Shader::~Shader()
{

}

void Shader::compile()
{
    glCompileShaderARB(mID);
}

void Shader::setSource(const char* filename)
{
    char currentLine[200];
    int numberOfStrings = 0;
    std::string source = "";
    std::ifstream fileStream (filename);

    if(fileStream.is_open())
    {
        while(fileStream.good())
        {
            fileStream.getline(currentLine, 200);
            source += ((std::string)(currentLine)+"\n");
            numberOfStrings++;
        }
    }
    const char* temp = source.c_str();
    fileStream.close();
    glShaderSourceARB(mID, 1, &temp, NULL);
}

int Shader::errorCheck()
{
    int maxLength;
    int isCompiled;
    char* infoLog;

    glGetShaderiv(mID, GL_COMPILE_STATUS, &isCompiled);

    glGetShaderiv(mID, GL_INFO_LOG_LENGTH, &maxLength);

    /* The maxLength includes the NULL character */
    infoLog = new char[maxLength];

    glGetShaderInfoLog(mID, maxLength, &maxLength, infoLog);

    printf(infoLog);

    delete [] infoLog;

    return isCompiled;

}

GLenum Shader::getID()
{
    return mID;
}
