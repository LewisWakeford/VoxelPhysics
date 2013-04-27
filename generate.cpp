#define GLEW_STATIC
#include <GL/glew.h>
#define GLFW_DLL
#include <GL/glfw.h>

#include <stdlib.h>
#include <string.h>

#include "generate.h"
#include "shader.h"
#include "console.h"

void generateVoxelData(std::string fragmentShader, std::string outputFile)
{
    if(GLEW_OK != glewInit())
    {
        consolePrint("GLEW Failed to initalise.");
    }

	if (glTexImage3D == NULL)
	{
	    consolePrint("Cannot bind glTexImage3D.");
	}

    //Create a 3D texture to render to:
    unsigned int textureName;
    unsigned int bufferName;

    GLfloat* textureData;

    glGenTextures(1, &textureName);
    glBindTexture(GL_TEXTURE_3D, textureName);

    textureData = new GLfloat[32 * 32 * 32 * 4];
    //memset(textureData, 0, (32 * 32 * 32 * sizeof(GLfloat)));

    glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA, 32, 32, 32, 0, GL_RGBA,
             GL_FLOAT, 0); //Generate blank 3D texture 32x32x32


    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    //Create a frame buffer
    glGenFramebuffers(1, &bufferName);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, bufferName);

    //Bind texture to frame buffer.
    glFramebufferTexture(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, textureName, 0);
    GLenum buffs[] = {GL_COLOR_ATTACHMENT0};
    glDrawBuffers(1, buffs);
    glReadBuffer(GL_NONE);


    if(glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        consolePrint("Frame Buffer failed to load.");
    }

    //Setup Shader

    const char* vert_src;
    const char* frag_src;


    GLenum program = glCreateProgramObjectARB();
    GLenum vert_shader = glCreateShaderObjectARB(GL_VERTEX_SHADER_ARB);
    GLenum geom_shader = glCreateShaderObjectARB(GL_GEOMETRY_SHADER_ARB);
    GLenum frag_shader = glCreateShaderObjectARB(GL_FRAGMENT_SHADER_ARB);

    //std::string vert_str = getSource("generate.vert");
    //std::string frag_str = getSource(fragmentShader.c_str());

    //vert_src = vert_str.c_str();
    //frag_src = frag_str.c_str();

    //glShaderSourceARB(vert_shader, 1, &vert_src, NULL);
    //glShaderSourceARB(frag_shader, 1, &frag_src, NULL);

    setSource(vert_shader, "generate.vert");
    setSource(geom_shader, "generate.geom");
    setSource(frag_shader, "generate_flat.frag");

    glCompileShaderARB(vert_shader);
    errorCheck(vert_shader);
    glCompileShaderARB(geom_shader);
    errorCheck(geom_shader);
    glCompileShaderARB(frag_shader);
    errorCheck(frag_shader);

    glAttachObjectARB(program, vert_shader);
    glAttachObjectARB(program, geom_shader);
    glAttachObjectARB(program, frag_shader);

    glLinkProgramARB(program);

    glUseProgramObjectARB(program);

    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    GLfloat vertexs[18] =
    {
        -1.0f, -1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f
    };
    glVertexPointer(3, GL_FLOAT, 0, vertexs);
    GLfloat colors[18] =
    {
        1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f,
        1.0f, 1.0f, 0.0f
    };
    glColorPointer(3, GL_FLOAT, 0, colors);


    glViewport(0,0,32,32);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    glDrawArrays(GL_TRIANGLES, 0, 6);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_COLOR_ARRAY);

    //glDrawArraysInstancedARB(GL_TRIANGLES, 0, 6, 32);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    glBindTexture(GL_TEXTURE_3D, textureName);
    glGetTexImage(GL_TEXTURE_3D, 0, GL_RGBA, GL_FLOAT, textureData);

    std::ofstream fileStream;
    fileStream.open(outputFile);
    for(int i = 0; i < (32 * 32 * 32 * 4) - 3; i+=4)
    {
        fileStream << (textureData[i]) << " " << (textureData[i+1]) << " " << (textureData[i+2]) << " " << (textureData[i+3]) << "   ";
    }
    fileStream.close();

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBindTexture(textureName, 0);
    glDisable(GL_TEXTURE_3D);
    glUseProgram(0);
}
