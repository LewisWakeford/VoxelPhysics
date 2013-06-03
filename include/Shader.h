#ifndef SHADER_H_INCLUDED
#define SHADER_H_INCLUDED

#include "VP.h"

#include <iostream>
#include <fstream>
#include <string>

/**
    Class: Shader
    Wrapper class for an OpenGL shader.
*/
class Shader
{
    public:
        Shader(GLenum shaderID);

        //TODO: Remove shader from OpenGL when this object is destoryed.
        virtual ~Shader();

        void setSource(const char* filename);

        void compile();

        int errorCheck();

        GLenum getID();

    protected:
        GLenum mID;


    private:
};


#endif
