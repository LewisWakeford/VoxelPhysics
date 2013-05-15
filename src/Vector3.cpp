#include "Vector3.h"
#include <cmath>

Vector3f::Vector3f()
{
    x = 0;
    y = 0;
    z = 0;
}

Vector3f::Vector3f(float xVal, float yVal, float zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

Vector3f::~Vector3f()
{
    //dtor
}

void Vector3f::set(float xVal, float yVal, float zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

void Vector3f::normalize()
{
    float mag = sqrtf((x * x) + (y*y) + (z*z));
    x = x/mag;
    y = y/mag;
    z = z/mag;
}

Vector3f Vector3f::normalized() const
{
    float mag = sqrtf((x * x) + (y*y) + (z*z));
    if (mag == 0) return Vector3f(0.0f, 0.0f, 0.0f);
    else return Vector3f(x/mag, y/mag, z/mag);
}

float Vector3f::dot(const Vector3f& other) const
{
    float dotProduct = (x * other.x) + (y*other.y) + (z*other.z);
/*
    for (int i = 0; i < 3; i++)
    {
        for (int k = 0; k < 3; k++)
        {
            dotProduct = dotProduct + (get(i)+other.get(k));
        }
    }*/
    return dotProduct;
}

void Vector3f::add(const Vector3f& vector)
{
    x += vector.x;
    y += vector.y;
    z += vector.z;
}

float Vector3f::get(unsigned int index) const
{
    switch(index)
    {
        case 0:
            return x;
            break;
        case 1:
            return y;
            break;
        case 2:
            return z;
            break;
    }
}

void Vector3f::set(unsigned int index, float value)
{
    switch(index)
    {
        case 0:
            x = value;
            break;
        case 1:
            y = value;
            break;
        case 2:
            z = value;
            break;
    }
}

float Vector3f::length() const
{
    float mag = sqrtf((x * x) + (y*y) + (z*z));
    return mag;
}

Vector3f Vector3f::mult(float ratio) const
{
    return Vector3f(x*ratio, y*ratio, z*ratio);
}

float Vector3f::distance(const Vector3f& other)
{
    float dX = x - other.x;
    float dY = y - other.y;
    float dZ = z - other.z;
    float distance = sqrtf((dX * dX) + (dY*dY) + (dZ*dZ));
    return distance;
}

Vector3f Vector3f::operator - () const
{
    return Vector3f(-x, -y, -z);
}

Vector3f Vector3f::operator - (const Vector3f& other) const
{
    return Vector3f(x-other.x, y-other.y, z-other.z);
}

Vector3f Vector3f::operator + (const Vector3f& other) const
{
    return Vector3f(x+other.x, y+other.y, z+other.z);
}

void Vector3f::operator += (const Vector3f& other)
{
    x += other.x;
    y += other.y;
    z += other.z;
}

void Vector3f:: operator = (const Vector3f& other)
{
    x = other.x;
    y = other.y;
    z = other.z;
}

Vector3f Vector3f::operator * (const float& ratio) const
{
    return Vector3f(x*ratio, y*ratio, z*ratio);
}

Vector3i::Vector3i()
{
    x = 0;
    y = 0;
    z = 0;
}

Vector3i::Vector3i(int xVal, int yVal, int zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

Vector3i::~Vector3i()
{
    //dtor
}

void Vector3i::set(int xVal, int yVal, int zVal)
{
    x = xVal;
    y = yVal;
    z = zVal;
}

void Vector3i::normalize()
{
    float mag = sqrtf((x * x) + (y*y) + (z*z));
    x = x/mag;
    y = y/mag;
    z = z/mag;
}

int Vector3i::get(unsigned int index)
{
    switch(index)
    {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;
    }
}

Vector3i Vector3i::operator + (const Vector3i& other) const
{
    return Vector3i(x+other.x, y+other.y, z+other.z);
}

bool Vector3i::operator == (const Vector3i& other) const
{
    return (x == other.x && y == other.y && z == other.z);
}
