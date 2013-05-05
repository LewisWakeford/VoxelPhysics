#include "Shader.h"
#include "console.h"

Shader::Shader(GLenum shaderType)
{
    mID = glCreateShader(shaderType);
}

Shader::~Shader()
{

}

void Shader::compile()
{
    glCompileShader(mID);
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
    glShaderSource(mID, 1, &temp, NULL);
}

int Shader::errorCheck()
{
    int maxLength;
    int isCompiled;
    char* infoLog;

    glGetShaderiv(mID, GL_COMPILE_STATUS, &isCompiled);

    glGetShaderiv(mID, GL_INFO_LOG_LENGTH, &maxLength);

    /* The maxLength includes the NULL character */
    if(maxLength > 0)
    {
        infoLog = new char[maxLength];

        int length;

        glGetShaderInfoLog(mID, maxLength, &length, infoLog);
        std::string infoString(infoLog, length);

        std::cout << infoLog << std::endl;

        delete [] infoLog;
    }

    return isCompiled;

}

GLenum Shader::getID()
{
    return mID;
}
