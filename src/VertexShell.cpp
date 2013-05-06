#include "VertexShell.h"

#include "App.h"

VertexShell::VertexShell(App* app)
{
}

VertexShell::~VertexShell()
{
    delete mBuffer;
}

void VertexShell::addVertex(Vector3f coord, Vector3f normal)
{
    mVertexArray.push_back(coord.x); mVertexArray.push_back(coord.y); mVertexArray.push_back(coord.z);
    mVertexArray.push_back(normal.x); mVertexArray.push_back(normal.y); mVertexArray.push_back(normal.z);
}

void VertexShell::addVertex(GLfloat cX, GLfloat cY, GLfloat cZ, GLfloat nX, GLfloat nY, GLfloat nZ)
{
    mVertexArray.push_back(cX); mVertexArray.push_back(cY); mVertexArray.push_back(cZ);
    mVertexArray.push_back(nX); mVertexArray.push_back(nY); mVertexArray.push_back(nZ);
}

void VertexShell::startWrite()
{
    mVertexArray.clear();
}

void VertexShell::endWrite()
{
    setBufferData();

    mVertexArray.clear();
}

Buffer* VertexShell::getBuffer()
{
    return mBuffer;
}

void VertexShell::createBuffer()
{
    mBuffer = new Buffer();
    mBuffer->init();
}

void VertexShell::setBufferData()
{
    createBuffer();

    mBuffer->setData(mVertexArray.data(), mVertexArray.size()/6, sizeof(float)*6, GL_STATIC_DRAW);
    mBuffer->setArray(0, 3, GL_FLOAT, 0, 0);
    mBuffer->setVertexArray(0);
    mBuffer->setArray(1, 3, GL_FLOAT, 3, 0);
    mBuffer->setNormalArray(1);

std::cout << "Number of vertices: " << mVertexArray.size()/6;

}

void VertexShell::setBuffer(Buffer* buffer)
{
    mBuffer = buffer;

    mBuffer->setArray(0, 3, GL_FLOAT, 0, 0);
    mBuffer->setVertexArray(0);
    mBuffer->setArray(1, 3, GL_FLOAT, 3, 0);
    mBuffer->setNormalArray(1);
}
