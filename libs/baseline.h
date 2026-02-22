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

private:
    std::vector<Node *> children;

public:
    Vector3D p0;
    Vector3D COM;
    const Body *body;
    double width;
    double mass;

    Node()
    {
        body = nullptr;
        width = 0;
        mass = 0;
        COM = Vector3D(0, 0, 0);
        children.assign(8, nullptr);
    }

    Node(Vector3D p, double node_width)
    {
        p0 = p;
        width = node_width;
        body = nullptr;
        mass = 0;
        COM = Vector3D(0, 0, 0);
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

    bool is_leaf() const
    {
        return children[0] == nullptr;
    }

    void make_children()
    {
        std::vector<Vector3D> p(8); // Requires default constructor
        double half_width = width * 0.5;
        Vector3D p00{p0.x, p0.y, p0.z};
        Vector3D p01{p0.x + half_width, p0.y, p0.z};
        Vector3D p02{p0.x, p0.y + half_width, p0.z};
        Vector3D p03{p0.x + half_width, p0.y + half_width, p0.z};
        Vector3D p10{p0.x, p0.y, p0.z + half_width};
        Vector3D p11{p0.x + half_width, p0.y, p0.z + half_width};
        Vector3D p12{p0.x, p0.y + half_width, p0.z + half_width};
        Vector3D p13{p0.x + half_width, p0.y + half_width, p0.z + half_width};

        p[0] = p00; // Choice p[0].p0 = Vector3D{p0.x, p0.y, p0.z}.. calls move assignment ?
        p[1] = p01;
        p[2] = p02;
        p[3] = p03;
        p[4] = p10;
        p[5] = p11;
        p[6] = p12;
        p[7] = p13;

        for (unsigned i = 0; i < 8; ++i)
        {
            children[i] = new Node(p[i], half_width);
        }
    }

    Node *get_child(const int idx) const
    {
        return children[idx];
    }

    void move_body_with_mass(int mov_idx)
    {
        children[mov_idx]->body = body;
        children[mov_idx]->COM = body->pos;
        children[mov_idx]->mass = body->mass;
        body = nullptr;
    }

    void move_body_ptrOnly(int mov_idx)
    {
        children[mov_idx]->body = body;
        body = nullptr;
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

    Vector3D compute_acceleration(const Body *const b, const Node *const n, const double theta);
    Statistics timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1);
    void insert(const Body *const b, Node *const n);

protected:
    virtual Node *construct_tree(const std::vector<Body> &Bodies);

public:
    Statistics run(std::vector<Body> &Bodies, const int num_steps);

    BaseSim(double dt, double theta) : dt(dt), theta(theta) {}
};

class PostCOMSim : public BaseSim
{
private:
    void compute_COM_post(Node *const n);
    void insert(const Body *const b, Node *const n);

protected:
    Node *construct_tree(const std::vector<Body> &Bodies) override;

public:
    PostCOMSim(double dt, double theta) : BaseSim(dt, theta) {}
};
