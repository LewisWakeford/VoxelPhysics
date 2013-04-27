#include "Mesh.h"

Mesh::Mesh()
{
    mType = 0;
    mGotVertices = GL_FALSE;
    mGotNormals = GL_FALSE;
    mGotTextures = GL_FALSE;
    mGotColors = GL_FALSE;
}

Mesh::~Mesh()
{
    //dtor
}

void Mesh::setVertices(std::vector<GLfloat> vertices, std::vector<GLushort> faces)
{
    mVertices = vertices;
    mFaces = faces;
    mGotVertices = GL_TRUE;
}

void Mesh::setNormals(std::vector<GLfloat> normals)
{
    mNormals = normals;
    mGotNormals = GL_TRUE;
}

void Mesh::setTextures(std::vector<GLfloat> texCoords, std::vector<GLint> textureIds)
{
    mTexCoords = texCoords;
    mTextureIds = textureIds;
    mType = mType | VP_MESH_TEXTURED;
    mGotTextures = GL_TRUE;
}

void Mesh::setColors(std::vector<GLfloat> colors, GLint size)
{
    mColors = colors;
    mColorSize = size;
    mType = mType | VP_MESH_COLORED;
    mGotColors = GL_TRUE;
}

void Mesh::setType(GLenum type)
{
    mType = type;
}

std::vector<GLfloat> Mesh::getVertices()
{
    return mVertices;
}

std::vector<GLushort> Mesh::getFaces()
{
    return mFaces;
}

std::vector<GLfloat> Mesh::getNormals()
{
    return mNormals;
}

std::vector<GLfloat> Mesh::getTexCoords()
{
    return mTexCoords;
}

std::vector<GLint> Mesh::getTextureIds()
{
    return mTextureIds;
}

std::vector<GLfloat> Mesh::getColors()
{
    return mColors;
}

GLint Mesh::getColorSize()
{
    return mColorSize;
}

GLenum Mesh::getType()
{
    return mType;
}

GLboolean Mesh::hasVertices()
{
    return mGotVertices;
}

GLboolean Mesh::hasNormals()
{
    return mGotNormals;
}

GLboolean Mesh::hasTextures()
{
    return mGotTextures;
}

GLboolean Mesh::hasColors()
{
    return mGotColors;
}
