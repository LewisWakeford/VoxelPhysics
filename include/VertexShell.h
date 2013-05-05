#ifndef VERTEXSHELL_H
#define VERTEXSHELL_H

#include "VP.h"
#include "Vector3.h"

#include <vector>

#include "Buffer.h"
#include "RigidBody.h"

class App;
class Matter;

class VertexShell
{
    public:
        VertexShell(App* app);
        virtual ~VertexShell();

        void addVertex(Vector3f coord, Vector3f normal);
        void addVertex(GLfloat cX, GLfloat cY, GLfloat cZ, GLfloat nX, GLfloat nY, GLfloat nZ);

        //Start writing to this vertex shell's vertex array
        void startWrite();

        //Stop writing to this vertex shell's vertex array and move data to VBO.
        void endWrite();

        Buffer* getBuffer();
        void setBuffer(Buffer* buffer);


    protected:
        std::vector<GLfloat> mVertexArray;

        //Create a VBO if required
        void createBuffer();

        //Write the data to the VBO
        void setBufferData();

        //Vertex buffer this shell uses.
        Buffer* mBuffer;


    private:
};

#endif // VERTEXSHELL_H
