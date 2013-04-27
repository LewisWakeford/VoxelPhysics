#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H

#include "VP.h"

#include "Shader.h"

class ShaderProgram
{
    public:
        ShaderProgram();
        virtual ~ShaderProgram();

        void attach(Shader shader);
        void link();
        void use();
        GLboolean checkLink();

        GLenum getID();

    protected:
        GLenum mID;

    private:
};

#endif // SHADERPROGRAM_H
