#ifndef DRAWCALL_H
#define DRAWCALL_H

#include "VP.h"

#include <vector>

class App;
class Mesh;
class Matrix4D;

class DrawCall
{
   public:
        DrawCall(App* app, GLint materialID);
        DrawCall(App* app, GLint materialID, GLuint bufferName);
        virtual ~DrawCall();

        GLint getMaterial();

        GLboolean drawMesh(Mesh& mesh, Matrix4D& matrix);
        void clear();

        GLboolean hasTexture();
        GLboolean hasNormals();

        /*
        Checks that this draw call is valid:
            -Must have at least 1 vertex.
            -Number of vertex coords must be multiple of 3.
            -Number of faces must be multiple of 3.
            -Number of normal coords must be equal to number of vertex coords or zero.
            -Number of texture coords must be equal to (num of vertex coords/3 * 2) or zero.
            -Number of color values must be equal to (num of vertex coords/3 * 4) or zero.
            -Both texture coords and color values cannot be zero.
        */

        GLboolean valid();


        GLboolean draw();

        GLint getFaceCount();

        void startWrite();

        void endWrite();

        void setVertexBuffer(GLuint bufferName, GLuint faceCount);
        void setMatrixBuffer(GLuint bufferName);

    protected:

        GLuint mFaceCount;

        void createBuffer();
        void setBufferData();

        std::vector<GLuint> mFaces;
        std::vector<GLfloat> mVertices;
        std::vector<GLfloat> mNormals;
        std::vector<GLfloat> mTexCoords;
        std::vector<GLfloat> mColors;

        GLint mMaterialID;
        GLuint mVertexBufferName;
        GLuint mMatrixBufferName;

        App* mApp;

};

#endif // DRAWCALL_H
