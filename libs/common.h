#pragma once
#define G 6.6743E-20 // km^3*kg^-1*s^-2

class Vector3D
{
public:
    double x, y, z;

    Vector3D(double init_x, double init_y, double init_z) : x(init_x), y(init_y), z(init_z) {}

    Vector3D() : x(0), y(0), z(0) {}

    Vector3D &operator+=(const Vector3D &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }
};

class Body
{
public:
    Vector3D pos, vel;
    double mass;
    int index; // Padding issues !

    Body(double mass, Vector3D pos, Vector3D vel, int index)
    {
        this->mass = mass;
        this->pos = pos; // Default copy construction !
        this->vel = vel;
        this->index = index;
    }

    // delete default constructor and copies ?

    Body() {}
};

class Statistics // Reporting Statistics
{
public:
    double t_insert;
    double t_reorder;
    double t_force;
    double t_leapfrog;

    Statistics() : t_insert(0), t_force(0), t_leapfrog(0), t_reorder(0) {}

    void print()
    {
        std::cout << "time taken by tree construction and deletion : " << t_insert << "\n";
        std::cout << "time taken by force computation : " << t_force << "\n";
        std::cout << "time taken by leapfrog integration : " << t_leapfrog << "\n";
        std::cout << "time taken by reorder : " << t_reorder << "\n";
    }
};
