#include "Material.h"

Material::Material()
{
    //Default Properties
    mColor[0] = 1.0f; mColor[1] = 1.0f; mColor[2] = 1.0f; //Red
    mDensity = 1.0f;
    mPressureLimit =    300.0f;
    mStressLimit =      150.0f;
    mMinStrength = 0.5f;
    mMaxStrength = 1.5f;
    mRandomChance = 0.0f;
}

Material::~Material()
{

}

const GLfloat* Material::getColor3() const
{
    return mColor;
}

float Material::getDensity() const
{
    return mDensity;
}
float Material::getPressureLimit() const
{
    return mPressureLimit;
}

float Material::getStressLimit() const
{
    return mStressLimit;
}

float Material::getMinStrength() const
{
    return mMinStrength;
}
float Material::getMaxStrength() const
{
    return mMaxStrength;
}

float Material::getRandomChance() const
{
    return mRandomChance;
}

bool Material::nonTrivial(float energy) const
{
    float lowestPressure = mMinStrength * mPressureLimit;
    float lowestStress = mMinStrength * mStressLimit;

    return (energy > lowestPressure || energy > lowestStress);
}
