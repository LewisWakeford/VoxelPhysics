#ifndef MESH_H
#define MESH_H

#include "VP.h"

#include <vector>



//Class: Mesh
//Description: Stores the information needed to render a mesh. Meshes are stored in the resource manager, Nodes that use meshs reference them through the resource manager.
//Multiple nodes with the same mesh do not need to store all the vertices again.
class Mesh
{
    public:
        Mesh();
        virtual ~Mesh();

        //Methods to change properties of mesh, most are self explanatory.
        void setVertices(std::vector<GLfloat> vertices, std::vector<GLushort> faces);
        void setNormals(std::vector<GLfloat> normals);
        void setTextures(std::vector<GLfloat> texCoords, std::vector<GLint> textureIds);
        void setColors(std::vector<GLfloat> colors, GLint size); //size is the number of color components: 3 = RGB, 4 = RGB + alpha.
        void setType(GLenum type);

        //Accessor Methods
        std::vector<GLfloat> getVertices();
        std::vector<GLushort> getFaces();
        std::vector<GLfloat> getNormals();
        std::vector<GLfloat> getTexCoords();
        std::vector<GLint> getTextureIds();
        std::vector<GLfloat> getColors();
        GLint getColorSize();

        GLenum getType();

        GLboolean hasVertices();
        GLboolean hasNormals();
        GLboolean hasTextures();
        GLboolean hasColors();

    protected:
        std::vector<GLfloat> mVertices;
        std::vector<GLushort> mFaces;
        std::vector<GLfloat> mNormals;
        std::vector<GLfloat> mTexCoords;
        std::vector<GLint> mTextureIds;
        std::vector<GLfloat> mColors;
        GLint mColorSize;

        //Booleans that describe mesh state of compeltion:
        GLboolean mGotVertices;
        GLboolean mGotNormals;
        GLboolean mGotTextures;
        GLboolean mGotColors;

        //The available render types of this mesh. Currently there are two: textured and colored. When the color/texture vectors are set this property is updated automatically.
        //Can also be set manually via setType().
        //When the mesh is rendered it will try to render itself textured, if not colored, though the render call can override the meshs settings.
        GLenum mType;

    private:
};

#endif // MESH_H
