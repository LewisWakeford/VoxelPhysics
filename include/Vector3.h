#ifndef VECTOR3_H
#define VECTOR3_H


class Vector3f
{
    public:
        Vector3f();
        Vector3f(float xVal, float yVal, float zVal);
        virtual ~Vector3f();

        float x, y, z;

        void set(float xVal, float yVal, float zVal);
        void set(unsigned int index, float value);
        float get(unsigned int index) const;
        void add(const Vector3f& otherVec);
        void normalize();
        Vector3f normalized() const;
        float dot(const Vector3f& other) const;
        float length() const;
        Vector3f mult(float ratio) const;
        float distance(const Vector3f& other);

        Vector3f operator - () const;
        Vector3f operator - (const Vector3f& other) const;
        Vector3f operator + (const Vector3f& other) const;
        void operator += (const Vector3f& other);
        Vector3f operator * (const float& ratio) const;

    protected:


    private:
};

class Vector3i
{
    public:
        Vector3i();
        Vector3i(int xVal, int yVal, int zVal);
        virtual ~Vector3i();

        int x, y, z;

        void set(int xVal, int yVal, int zVal);
        int get(unsigned int index);
        void normalize();

        Vector3i operator + (const Vector3i& other) const;
        bool operator == (const Vector3i& other) const;

    protected:


    private:
};

#endif // VECTOR3_H
