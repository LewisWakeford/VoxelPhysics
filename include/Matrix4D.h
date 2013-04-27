#ifndef MATRIX4D_H
#define MATRIX4D_H

#include "VP.h"
#include <boost/shared_ptr.hpp>
#include "Vector3.h"

/**
    Class: Matrix4D
    Description: Wrapper class for 4x4 matrix.
        Matrix is column major where the rows represent axes in the order x, y, z from top to bottom.
*/

class Matrix4D
{
    private:
        //Column Major 4x4 Matrix
        float mValues[16];

    public:
        Matrix4D();
        Matrix4D(const float* values);
        virtual ~Matrix4D();

        //Set the value at the specific row/column space.
        void set(int row, int column, float value);

        //Set the values for an entire row. col0 is the left most Column, col3 is the right most.
        //row should be an integer 0 - 3.
        void setRow(int row, float col0, float col1, float col2, float col3);

        //Set the values for an entire column. row0 is the top row, row3 is the bottom one.
        //column should be an integer 0 - 3.
        void setColumn(int column, float row0, float row1, float row2, float row3);

        //---Matrix math operations--

        //Multiply this matrix by another matrix.
        //Order is: this matrix * other matrix
        void multiplyBy(Matrix4D* otherMatrix);

        //Rotate the transform by x in the x-axis, y in the y-axis and z in the z-axis.
        void rotate(float x, float y, float z);

        //Translate the transform by x in the x-axis, y in the y-axis and z in the z-axis.
        void translate(float x, float y, float z);

        //Scale the transform in every axis by scale.
        void scale(float scale);

        //Scale the transform by x in the x-axis, y in the y-axis and z in the z-axis.
        void scale(float x, float y, float z);

        //Invert the matrix.
        bool invert();
        Matrix4D inverted() const;

        //Transform a vertex through the matrix. Does not modify original data.
        Vertex3 transformVertex(Vertex3 vertex) const;

        Vector3f transformVertex(Vector3f vector) const;

        Vertex3 translateVertex(Vertex3 vertex);
        //Rotate a vector through the matrix, but do not translate it. Useful for normals as they should always be relative to their vertex.
        Vector3f rotateVector(Vector3f vector) const;

        //Static matrix math operations.
        //Usage example:
        //Matrix 4D ident = Matrix4D::createIdentity();

        //Create an identiy matrix.
        static Matrix4D createIdentity();

        //Create rotation matrix.
        static Matrix4D createRotation(float x, float y, float z);
        static Matrix4D createXRotation(float x);
        static Matrix4D createYRotation(float y);
        static Matrix4D createZRotation(float z);
        static Matrix4D createArbRotation(Vector3f axis, float angle);

        //Create translation matrix.
        static Matrix4D createTranslation(float x, float y, float z);

        //Create scale matrix.
        static Matrix4D createScale(float scale);
        static Matrix4D createScale(float x, float y, float z);

        //Create an easy transform. The node with this transform will appear at (xPos, yPos, zPos) rotated to match pitch, yaw and roll.
        static Matrix4D createEasyTransform(float xPos, float yPos, float zPos,
                                        float pitch, float yaw, float roll);

        //Return the values stored in this matrix.
        float* getValues();

    protected:
};

typedef boost::shared_ptr<Matrix4D> Matrix4Dptr;

#endif // MATRIX4D_H
