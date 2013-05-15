#ifndef MATERIAL_H
#define MATERIAL_H

#include "VP.h"
#include <string>
#include <boost/shared_ptr.hpp>

/*
    Property set of a material.
    Each matter chunk has a material that defines its physical behaviour and visual appearance.
*/
class Material
{
    public:
        Material();
        virtual ~Material();

        const GLfloat* getColor3() const;

        float getDensity() const;

        float getPressureLimit() const;
        float getPressureGrouping() const;
        float getDebrisPressureLimit() const;
        float getStressLimit() const;
        float getStressGrouping() const;
        float getDebrisStressLimit() const;
        float getBondStrength() const;
        float getFriction() const;
        float getRestitution() const;

        float getMinStrength() const;
        float getMaxStrength() const;

        void setDensity(float density);
        void setPressureLimit(float limit);
        void setStressLimit(float limit);
        void setColor(float red, float green, float blue);
        void setBondStrength(float bondStrength);
        void setStrength(float min, float max);


        int getTrivialMass() const;

        //Returns true if this amount of energy can possibly alter the shape of an object composed of this material.
        bool nonTrivial(float energy) const;

    protected:

        //Visual
        //Just Color for now, maybe add textures and whatnot later.
        GLfloat mColor[3];

        //Physical
        float mDensity; // Mass over Volume kg/m^3 so with density 1 and voxel spacing of 1, mass will be equal to num voxels.

        float mPressureLimit; // Amount of pressure a voxel can withstand before broken.

        float mBondStrength;    //The amount of
        float mPressureGrouping; //The amount of pressure adjanct broken nodes can share to group up.
        float mDebrisPressureLimit; //The most total pressure a piece of debris can contain. More pressure = greater fragmentation.
        float mStressLimit; // Amount of stress a voxel can withstand before breaking.
        float mStressGrouping; //The amount of pressure adjanct snapped voxels can share.
        float mDebrisStressLimit; //The most total stress a piece of stress can contain. More stress = greater stress.

        //Note: Pressure and stress limits are multiplied by strength.
        float mMinStrength; //The minimum amount of strength a particular voxel can be randomly assigned. Should be 1 or less.
        float mMaxStrength; //The maximum amount of strength a particular voxel can be randomly assigned. Should be 1 or more.

        float mFriction;
        float mRestitution;

        float mTrivialThreshold;
        int mTrivialMass;

    private:
};

typedef boost::shared_ptr<Material> MaterialPtr;

#endif // MATERIAL_H
