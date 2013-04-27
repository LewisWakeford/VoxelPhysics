#include "Matrix4D.h"
#include <math.h>
/**
    For function descriptions see header file.
*/

Matrix4D::Matrix4D()
{

}

Matrix4D::Matrix4D(const float* values)
{
    for(int i = 0; i < 16; i++)
    {
        mValues[i] = values[i];
    }
}

Matrix4D::~Matrix4D()
{

}

Vertex3 Matrix4D::transformVertex(Vertex3 vertex) const
{
    Vertex3 result;
    result.x =  vertex.x*mValues[0] +
                vertex.y*mValues[4] +
                vertex.z*mValues[8] +
                mValues[12];

    result.y =  vertex.x*mValues[1] +
                vertex.y*mValues[5] +
                vertex.z*mValues[9] +
                mValues[13];

    result.z =  vertex.x*mValues[2] +
                vertex.y*mValues[6] +
                vertex.z*mValues[10] +
                mValues[14];

    return result;
}

Vector3f Matrix4D::transformVertex(Vector3f vertex) const
{
    Vector3f result;
    result.x =  vertex.x*mValues[0] +
                vertex.y*mValues[4] +
                vertex.z*mValues[8] +
                mValues[12];

    result.y =  vertex.x*mValues[1] +
                vertex.y*mValues[5] +
                vertex.z*mValues[9] +
                mValues[13];

    result.z =  vertex.x*mValues[2] +
                vertex.y*mValues[6] +
                vertex.z*mValues[10] +
                mValues[14];

    return result;
}

Vector3f Matrix4D::rotateVector(Vector3f vector) const
{
    Vector3f result;
    result.x =  vector.x*mValues[0] +
                vector.y*mValues[4] +
                vector.z*mValues[8];

    result.y =  vector.x*mValues[1] +
                vector.y*mValues[5] +
                vector.z*mValues[9];

    result.z =  vector.x*mValues[2] +
                vector.y*mValues[6] +
                vector.z*mValues[10];

    return result;
}

void Matrix4D::setRow(int row, float col0, float col1, float col2, float col3)
{
    mValues[row] = col0;
    mValues[row+4] = col1;
    mValues[row+8] = col2;
    mValues[row+12] = col3;
}

void Matrix4D::setColumn(int column, float row0, float row1, float row2, float row3)
{
    mValues[(column * 4)] = row0;
    mValues[(column * 4)+1] = row1;
    mValues[(column * 4)+2] = row2;
    mValues[(column * 4)+3] = row3;
}

void Matrix4D::multiplyBy(Matrix4D* otherMatrix)
{
    float newValues[16];

    //I is row
    for(int i = 0; i < 4; i++)
    {
        //J is column
        for(int j = 0; j < 4; j++)
        {
            newValues[(j*4)+i] = 0;
            for(int k = 0; k < 4; k++)
            {
                newValues[(j*4)+i] += mValues[(k*4)+i]*otherMatrix->mValues[(j*4)+k];
            }
        }
    }

    for(int i = 0; i < 15; i++)
    {
        mValues[i] = newValues[i];
    }

}

void Matrix4D::rotate(float x, float y, float z)
{
    Matrix4D rotMat = Matrix4D::createRotation(x, y, z);
    multiplyBy(&rotMat);
}

void Matrix4D::translate(float x, float y, float z)
{
    Matrix4D transMat = Matrix4D::createTranslation(x, y, z);
    multiplyBy(&transMat);
}

void Matrix4D::scale(float aScale)
{
    Matrix4D scaleMat = Matrix4D::createScale(aScale);
    multiplyBy(&scaleMat);
}

void Matrix4D::scale(float x, float y, float z)
{
    Matrix4D scaleMat = Matrix4D::createScale(x, y, z);
    multiplyBy(&scaleMat);
}

bool Matrix4D::invert()
{
    //Base formula taken from:
    //http://www.cg.info.hiroshima-cu.ac.jp/~miyzaki/knowledge/teche23.html

    //Inverse = 1/determinant * adjoint of matrix
    float adjoint[16]; //Adjoint of matrix
    float det; //Determinant

    adjoint[0] =    mValues[5]  * mValues[10] * mValues[15] +
                    mValues[9]  * mValues[7]  * mValues[14] +
                    mValues[13] * mValues[6]  * mValues[11] -
                    mValues[5]  * mValues[11] * mValues[14] -
                    mValues[9]  * mValues[6]  * mValues[15] -
                    mValues[13] * mValues[7]  * mValues[10];

    adjoint[4] =    mValues[4]  * mValues[11] * mValues[14] +
                    mValues[8]  * mValues[6]  * mValues[15] +
                    mValues[12] * mValues[7]  * mValues[10] -
                    mValues[4]  * mValues[10] * mValues[15] -
                    mValues[8]  * mValues[7]  * mValues[14] -
                    mValues[12] * mValues[6]  * mValues[11];

    adjoint[8] =    mValues[4]  * mValues[9]  * mValues[15] +
                    mValues[8]  * mValues[7]  * mValues[13] +
                    mValues[12] * mValues[5]  * mValues[11] -
                    mValues[4]  * mValues[11] * mValues[13] -
                    mValues[8]  * mValues[5]  * mValues[15] -
                    mValues[12] * mValues[7]  * mValues[9];

    adjoint[12] =   mValues[4]  * mValues[10] * mValues[13] +
                    mValues[8]  * mValues[5]  * mValues[14] +
                    mValues[12] * mValues[6]  * mValues[9]  -
                    mValues[4]  * mValues[9]  * mValues[14] -
                    mValues[8]  * mValues[6]  * mValues[13] -
                    mValues[12] * mValues[5]  * mValues[10];


    adjoint[1] =    mValues[1]  * mValues[11] * mValues[14] +
                    mValues[9]  * mValues[2]  * mValues[15] +
                    mValues[13] * mValues[3]  * mValues[10] -
                    mValues[1]  * mValues[10] * mValues[15] -
                    mValues[9]  * mValues[3]  * mValues[14] -
                    mValues[13] * mValues[2]  * mValues[11];

    adjoint[5] =    mValues[0]  * mValues[10] * mValues[15] +
                    mValues[8]  * mValues[3]  * mValues[14] +
                    mValues[12] * mValues[2]  * mValues[11] -
                    mValues[0]  * mValues[11] * mValues[14] -
                    mValues[8]  * mValues[2]  * mValues[15] -
                    mValues[12] * mValues[3]  * mValues[10];

    adjoint[9] =    mValues[0] * mValues[11]  * mValues[13] +
                    mValues[8]  * mValues[1]  * mValues[15] +
                    mValues[12] * mValues[3]  * mValues[9]  -
                    mValues[0]  * mValues[9]  * mValues[15] -
                    mValues[8]  * mValues[3]  * mValues[13] -
                    mValues[12] * mValues[1]  * mValues[11] ;

    adjoint[13] =   mValues[0]  * mValues[9] * mValues[14]  +
                    mValues[8]  * mValues[2] * mValues[13]  +
                    mValues[12] * mValues[1] * mValues[10]  -
                    mValues[0]  * mValues[10] * mValues[13] -
                    mValues[8]  * mValues[1] * mValues[14]  -
                    mValues[12] * mValues[2] * mValues[9];

    adjoint[2] =    mValues[1]  * mValues[6] * mValues[15]  +
                    mValues[5]  * mValues[3] * mValues[14]  +
                    mValues[13] * mValues[2] * mValues[7]   -
                    mValues[1]  * mValues[7] * mValues[14]  -
                    mValues[5]  * mValues[2] * mValues[15]  -
                    mValues[13] * mValues[3] * mValues[6];

    adjoint[6] =    mValues[0]  * mValues[7] * mValues[14]  +
                    mValues[4]  * mValues[2] * mValues[15]  +
                    mValues[0]  * mValues[6] * mValues[15]  -
                    mValues[12] * mValues[3] * mValues[6]   -
                    mValues[4]  * mValues[3] * mValues[14]  -
                    mValues[12] * mValues[2] * mValues[7];

    adjoint[10] =   mValues[0]  * mValues[5] * mValues[15]  +
                    mValues[4]  * mValues[3] * mValues[13]  +
                    mValues[12] * mValues[1] * mValues[7]   -
                    mValues[0]  * mValues[7] * mValues[13]  -
                    mValues[4]  * mValues[1] * mValues[15]  -
                    mValues[12] * mValues[3] * mValues[5];

    adjoint[14] =   mValues[0]  * mValues[6] * mValues[13] +
                    mValues[4]  * mValues[1] * mValues[14] +
                    mValues[12] * mValues[2] * mValues[5]  -
                    mValues[0]  * mValues[5] * mValues[14] -
                    mValues[4]  * mValues[2] * mValues[13] -
                    mValues[12] * mValues[1] * mValues[6];

    adjoint[3] =    mValues[1] * mValues[7] * mValues[10]   +
                    mValues[5] * mValues[2] * mValues[11]   +
                    mValues[9] * mValues[3] * mValues[6]    -
                    mValues[5] * mValues[3] * mValues[10]   -
                    mValues[9] * mValues[2] * mValues[7]    -
                    mValues[1] * mValues[6] * mValues[11];

    adjoint[7] =    mValues[4] * mValues[2] * mValues[11]   +
                    mValues[4] * mValues[3] * mValues[10]   +
                    mValues[8] * mValues[2] * mValues[7]    -
                    mValues[0] * mValues[6] * mValues[11]   -
                    mValues[0] * mValues[7] * mValues[10]   -
                    mValues[8] * mValues[3] * mValues[6];

    adjoint[11] =   mValues[0] * mValues[7] * mValues[9]    +
                    mValues[4] * mValues[1] * mValues[11]   +
                    mValues[8] * mValues[3] * mValues[5]    -
                    mValues[4] * mValues[3] * mValues[9]    -
                    mValues[8] * mValues[1] * mValues[7]    -
                    mValues[0] * mValues[5] * mValues[11];

    adjoint[15] =   mValues[0] * mValues[5] * mValues[10]   +
                    mValues[4] * mValues[2] * mValues[9]    +
                    mValues[8] * mValues[1] * mValues[6]    -
                    mValues[0] * mValues[6] * mValues[9]    -
                    mValues[4] * mValues[1] * mValues[10]   -
                    mValues[8] * mValues[2] * mValues[5];

    //The adjoint and the determinant have similar elements to their formulas.
    //If you look at the formulas in the link, note that the first and fifth rows of the determinant
    //are the same as the first element of the adjoint if they where multiplied by the first element of the original matrix.
    //This pattern repeats several times.

    det = mValues[0] * adjoint[0] + mValues[1] * adjoint[4] + mValues[2] * adjoint[8] + mValues[3] * adjoint[12];

    //If the adjoint is 0 we cannot actually create an inverse.
    //Should never really happen, but just in case.
    if (det == 0)
    {
        return false;
    }

    //Multiply adjoint by 1/determinant to get inverese.
    for (int i = 0; i < 16; i++)
    {
        mValues[i] = adjoint[i] * 1.0f / det;
    }
    return true;
}

Matrix4D Matrix4D::inverted() const
{
    Matrix4D matrix(mValues);
    matrix.invert();
    return matrix;
}

Matrix4D Matrix4D::createIdentity()
{
    Matrix4D matrix;
    matrix.setRow(0, 1, 0, 0, 0);
    matrix.setRow(1, 0, 1, 0, 0);
    matrix.setRow(2, 0, 0, 1, 0);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createXRotation(float x)
{
    Matrix4D matrix;
    matrix.setRow(0, 1, 0, 0, 0);
    matrix.setRow(1, 0, cos(x), -sin(x), 0);
    matrix.setRow(2, 0, sin(x), cos(x), 0);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createYRotation(float y)
{
    Matrix4D matrix;
    matrix.setRow(0, cos(y), 0, sin(y), 0);
    matrix.setRow(1, 0, 1, 0, 0);
    matrix.setRow(2, -sin(y), 0, cos(y), 0);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createZRotation(float z)
{
    Matrix4D matrix;
    matrix.setRow(0, cos(z), -sin(z), 0, 0);
    matrix.setRow(1, sin(z), cos(z), 0, 0);
    matrix.setRow(2, 0, 0, 1, 0);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createRotation(float x, float y, float z)
{
    Matrix4D xRot = Matrix4D::createXRotation(x);
    Matrix4D yRot = Matrix4D::createYRotation(y);
    Matrix4D zRot = Matrix4D::createZRotation(z);
    yRot.multiplyBy(&zRot);
    xRot.multiplyBy(&yRot);
    return xRot;
}

Matrix4D Matrix4D::createArbRotation(Vector3f axis, float angle)
{
    Matrix4D matrix;

    float cosAngle = cos(angle);
    float sinAngle = sin(angle);

    matrix.setRow(0, cosAngle+(axis.x*axis.x)*(1 - cosAngle),       axis.x*axis.y*(1-cosAngle)-axis.z*sinAngle,     axis.x*axis.z*(1 - cosAngle) + axis.y*sinAngle, 0);
    matrix.setRow(1, axis.x*axis.y*(1-cosAngle)+axis.z*sinAngle,    cosAngle+(axis.y*axis.y)*(1 - cosAngle),       axis.y*axis.z*(1-cosAngle)-axis.x*sinAngle,      0);
    matrix.setRow(2, axis.z*axis.x*(1-cosAngle)-axis.y*sinAngle,    axis.z*axis.y*(1-cosAngle)+axis.x*sinAngle,     cosAngle+(axis.z*axis.z)*(1 - cosAngle),        0);
    matrix.setRow(3, 0, 0, 0, 1);

    return matrix;
}

Matrix4D Matrix4D::createTranslation(float x, float y, float z)
{
    Matrix4D matrix;
    matrix.setRow(0, 1, 0, 0, x);
    matrix.setRow(1, 0, 1, 0, y);
    matrix.setRow(2, 0, 0, 1, z);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createScale(float scale)
{
    Matrix4D matrix;
    matrix.setRow(0, scale, 0, 0, 0);
    matrix.setRow(1, 0, scale, 0, 0);
    matrix.setRow(2, 0, 0, scale, 0);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createScale(float x, float y, float z)
{
    Matrix4D matrix;
    matrix.setRow(0, x, 0, 0, 0);
    matrix.setRow(1, 0, y, 0, 0);
    matrix.setRow(2, 0, 0, z, 0);
    matrix.setRow(3, 0, 0, 0, 1);
    return matrix;
}

Matrix4D Matrix4D::createEasyTransform(float xPos, float yPos, float zPos,
                                        float pitch, float yaw, float roll)
{
    Matrix4D rotMatrix = Matrix4D::createIdentity();
    Matrix4D pitchMat = Matrix4D::createXRotation(pitch);
    Matrix4D yawMat = Matrix4D::createYRotation(yaw);
    Matrix4D rollMat = Matrix4D::createZRotation(roll);

    yawMat.multiplyBy(&pitchMat);

    rotMatrix = yawMat;

    rotMatrix.mValues[12] = xPos;
    rotMatrix.mValues[13] = yPos;
    rotMatrix.mValues[14] = zPos;

    return rotMatrix;
}

float* Matrix4D::getValues()
{
    float* values = new float[16];
    int index = 0;
    for(int i = 0; i < 16; i++)
    {
            values[index] = mValues[i];
            index++;
    }
    return values;
}
