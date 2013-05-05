#ifndef BUFFER_H
#define BUFFER_H

#include <vector>

#include "VP.h"

//The SubArrray struct contains the properties of a single interleaved array in a buffer.
struct SubArray
{
    unsigned int mName; //The name of this array, for future reference.
    unsigned int mElementsPerVertex; //Order of this array in the bufffer.
    unsigned int mElementType; //Size of each object in the array. This is each float or byte for example, not each entire structure.
    unsigned int mOffset; //Offset from the start of the array.
    unsigned int mVertexAttrib; //The number of the custom vertex attribute this array represents.
};

class Buffer
{
    public:
        Buffer();
        virtual ~Buffer();

        void init(); //Generate the buffer if necessary, clear if already inited.
        void setData(void* dataPointer, unsigned int itemCount, unsigned int itemSize, unsigned int bufferType); //Set the buffer data.
        void zeroData(unsigned int itemCount, unsigned int itemSize, unsigned int bufferType);
        void setArray(unsigned int arrayName, unsigned int elementsPerVertex, unsigned int elementType, unsigned int offset, unsigned int vertexAttrib); //Create a sub array in this buffer.
        void clearArrays();

        void setVertexArray(unsigned int vertexArrayName);
        void setNormalArray(unsigned int normalArrayName);
        void setTexCoordArray(unsigned int texCoordArrayName);
        void setColorArray(unsigned int colorArrayName);

        void bind(); //Bind the VBO.
        void unbind();
        bool isBound(); //Check buffer is bound.

        void render();

        void bindArrays(); //Bind all arrrays to their correct targets.
        //void bindVertexArray();
        //void bindNormalArray();
        //void bindTexCoordArray();
        //void bindColorArray();
        //void bindCustomArrays();

        unsigned int getName();

    protected:
        static int sBoundBuffer; //The currently bound buffer.
        unsigned int mName; //The name of the VBO.

        unsigned int mItemCount; //The number of objects in the buffer.
        unsigned int mItemSize; //The size of the objects stored in the buffer, in bytes. Should be total of all the array's size properties.

        //These vectors map array names to their properties and allow the buffer to remember sub arrays.
        std::vector<SubArray> mArrayProperties;

        //The names of the "main" arrays. If these are non-zero, they will be bound when bindArrays is called.
        //If the array struct with one of these names has a custom attribute, it will be overwritten by the default arrays.
        unsigned int mVertexArrayName;
        unsigned int mNormalArrayName;
        unsigned int mTexCoordArrayName;
        unsigned int mColorArrayName;


    private:
};

#endif // BUFFER_H
