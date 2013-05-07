#include "Material.h"

Material::Material()
{
    //Default Properties
    mColor[0] = 1.0f; mColor[1] = 1.0f; mColor[2] = 1.0f; //Red
    mDensity = 5.0f;
    mPressureLimit =    10000.0f;
    mPressureGrouping = 5000.0f;
    mDebrisPressureLimit = 10000.0f;

    mStressLimit =      10000.0f;
    mStressGrouping = 10000.0f;
    mDebrisStressLimit = 25000.0f;

    mMinStrength = 0.5f;
    mMaxStrength = 1.5f;
    mRandomChance = 0.5f;
    mTrivialThreshold = 100000.0f;
    mTrivialMass = 9; //Minimum number of voxels to be considered complex enough for destruction.
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

float Material::getPressureGrouping() const
{
    return mPressureGrouping;
}

float Material::getDebrisPressureLimit() const
{
    return mDebrisPressureLimit;
}

float Material::getStressLimit() const
{
    return mStressLimit;
}

float Material::getStressGrouping() const
{
    return mStressGrouping;
}

float Material::getDebrisStressLimit() const
{
    return mDebrisStressLimit;
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
    return energy > mTrivialThreshold;
}

int Material::getTrivialMass() const
{
    return mTrivialMass;
}

void Material::setDensity(float density)
{
    mDensity = density;
}

void Material::setPressureLimit(float limit)
{
    mPressureLimit = limit;
}
