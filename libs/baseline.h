#pragma once
#include <vector>
#include <iostream>

#define G 6.6743E-20 // km^3*kg^-1*s^-2

class Vector3D
{
public:
    double x, y, z;

    Vector3D(double init_x, double init_y, double init_z) : x(init_x), y(init_y), z(init_z) {}

    Vector3D() : x(0), y(0), z(0) {}
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
};

class Node
{
public:
    Vector3D p0;
    Vector3D COM;
    const Body *body;
    double width;
    double mass;
    std::vector<Node *> children;

    Node()
    {
        body = nullptr;
        width = 0;
        children.assign(8, nullptr);
    }

    void reset_if_root()
    {
        // call only on root ?
        // Keeping width intact
        COM = Vector3D(0, 0, 0);
        body = nullptr;
        mass = 0; // will call isnert on this again from scratch

        for (Node *child : children)
        {
            delete child; // This doesn't set the pointer back to null... Checkout unique pointer semantics !
        }

        children.assign(8, nullptr);
    }
    ~Node()
    {
        for (Node *child : children)
        {
            delete child;
        }
    }
};

class Statistics // Reporting Statistics
{
public:
    double t_insert;
    double t_force;
    double t_leapfrog;

    Statistics() : t_insert(0), t_force(0), t_leapfrog(0) {}

    void print()
    {
        std::cout << "time taken by tree construction and deletion : " << t_insert << "\n";
        std::cout << "time taken by force computation : " << t_force << "\n";
        std::cout << "time taken by leapfrog integration : " << t_leapfrog << "\n";
    }
};

class BaseSim
{
private:
    const double dt;
    const double theta;
    void insert(const Body *const b, Node *const n);
    void make_children(Node *n);
    Node *construct_tree(const std::vector<Body> &Bodies);
    Vector3D compute_acceleration(const Body *const b, const Node *const n, const double theta);
    Statistics timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1);

public:
    Statistics run(std::vector<Body> &Bodies, const int num_steps);

    BaseSim(double dt, double theta) : dt(dt), theta(theta) {}
};

Vector3D compute_acceleration(const Body *const b, const Node *const n, const double theta);
Statistics timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, Node *root, const double dt, const double theta, unsigned i1);
