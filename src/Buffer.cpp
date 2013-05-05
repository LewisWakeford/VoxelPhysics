#include "Buffer.h"

int Buffer::sBoundBuffer = -1;

Buffer::Buffer()
{
    //Set all array names to zero to show that they are not yet valid.
    mVertexArrayName = 0;
    mNormalArrayName = 0;
    mTexCoordArrayName = 0;
    mColorArrayName = 0;
}

Buffer::~Buffer()
{
    glDeleteBuffers(1, &mName);
}

void Buffer::init()
{
    glGenBuffers(1, &mName);
}

void Buffer::bind()
{
    if(sBoundBuffer != mName)
    {
        glBindBuffer(GL_ARRAY_BUFFER, mName);
        sBoundBuffer = mName;
    }
}

void Buffer::unbind()
{
    if(sBoundBuffer == mName)
    {
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        sBoundBuffer = 0;
    }
}

bool Buffer::isBound()
{
    return (sBoundBuffer == mName);
}

void Buffer::setData(void* dataPointer, unsigned int itemCount, unsigned int itemSize, unsigned int bufferType)
{
    bind();

    glBufferData(GL_ARRAY_BUFFER, itemCount*itemSize, dataPointer, bufferType);

    mItemCount = itemCount;
    mItemSize = itemSize;

    unbind();
}

void Buffer::zeroData(unsigned int itemCount, unsigned int itemSize, unsigned int bufferType)
{
    glBufferData(GL_ARRAY_BUFFER, itemCount * itemSize, 0, bufferType);
    mItemCount = itemCount;
    mItemSize = itemSize;
}

void Buffer::setArray(unsigned int arrayName, unsigned int elementsPerVertex, unsigned int elementType, unsigned int offset, unsigned int vertexAttrib)
{
    SubArray array = {arrayName, elementsPerVertex, elementType, offset, vertexAttrib};
    mArrayProperties.push_back(array);
}

void Buffer::clearArrays()
{
    mArrayProperties.clear();
}

void Buffer::setVertexArray(unsigned int vertexArrayName)
{
    mVertexArrayName = vertexArrayName;
}
void Buffer::setNormalArray(unsigned int normalArrayName)
{
    mNormalArrayName = normalArrayName;
}

void Buffer::setTexCoordArray(unsigned int texCoordArrayName)
{
    mTexCoordArrayName = texCoordArrayName;
}

void Buffer::setColorArray(unsigned int colorArrayName)
{
    mColorArrayName = colorArrayName;
}

void Buffer::bindArrays()
{

    for(unsigned int i = 0; i < mArrayProperties.size(); i++)
    {
        SubArray* curr = &mArrayProperties[i];
        if(curr->mName == mVertexArrayName)
        {
            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(curr->mElementsPerVertex, curr->mElementType, mItemSize, (const GLvoid*)curr->mOffset);
        }
        else if(curr->mName == mNormalArrayName)
        {
            glEnableClientState(GL_NORMAL_ARRAY);
            glNormalPointer(curr->mElementType, mItemSize, (const GLvoid*)curr->mOffset);
        }
        else if(curr->mName == mTexCoordArrayName)
        {
            glEnableClientState(GL_TEXTURE_COORD_ARRAY);
            glTexCoordPointer(curr->mElementsPerVertex, curr->mElementType, mItemSize, (const GLvoid*)curr->mOffset);
        }
        else if(curr->mName == mColorArrayName)
        {
            glEnableClientState(GL_COLOR_ARRAY);
            glColorPointer(curr->mElementsPerVertex, curr->mElementType, mItemSize, (const GLvoid*)curr->mOffset);
        }
        else
        {
            glEnableVertexAttribArray(curr->mVertexAttrib);
            glVertexAttribIPointer(curr->mVertexAttrib, curr->mElementsPerVertex, curr->mElementType, mItemSize, (const GLvoid*)curr->mOffset);
        }
    }

}

unsigned int Buffer::getName()
{
    return mName;
}

void Buffer::render()
{
    bind();
    bindArrays();
    glDrawArrays(GL_TRIANGLES, 0, mItemCount);
    unbind();
}
