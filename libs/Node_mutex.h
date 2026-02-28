#pragma once
#include "common.h"
#include <vector>

/*
# Warning same class defined 3 times. Usually results compile error
Source should include only 1 node class file at any time

*/

class Node
{

private:
    Node *children; // this is a pointer to an array  !

public:
    Vector3D p0;
    Vector3D COM;
    const Body *body;
    double width;
    double mass;
    std::mutex w_lock; // Completely on the user side handled

    Node()
    {
        body = nullptr;
        width = 0;
        mass = 0;
        COM = Vector3D(0, 0, 0);
        children = nullptr;
    }

    Node(Vector3D p, double node_width)
    {
        p0 = p;
        width = node_width;
        body = nullptr;
        mass = 0;
        COM = Vector3D(0, 0, 0);
        children = nullptr;
    }

    bool is_leaf() const
    {
        return children == nullptr; // Is atomic necessary here ?
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

        children = new Node[8]; // calls default constructor !
        for (unsigned i = 0; i < 8; ++i)
        {
            children[i].p0 = p[i];
            children[i].width = half_width;
        }
    }

    Node *get_child(const int idx) const
    {
        return children + idx;
    }

    void move_body_with_mass(int mov_idx)
    {
        // Someone might also be doing operations on children !
        children[mov_idx].body = body;
        children[mov_idx].COM = body->pos;
        children[mov_idx].mass = body->mass;
        body = nullptr;
    }

    void move_body_ptrOnly(int mov_idx)
    {
        children[mov_idx].body = body;
        body = nullptr;
    }

    ~Node()
    {
        delete[] children;
    }
};
