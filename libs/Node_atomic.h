#pragma once
#include "common.h"
#include <vector>
#include <atomic>
/*
# Warning same class defined 3 times. Usually results compile error
Source should include only 1 node class file at any time

*/

class Node; // A forward declaration

struct alignas(16) Body_children
{
    const Body *b;
    Node *children;

    // Default constructor is default
    Body_children() = default;
    Body_children(const Body *new_b, Node *new_children) : b(new_b), children(new_children) {}

    bool operator==(const Body_children &other) const
    {
        return b == other.b && children == other.children;
    }
};

class Node
{

private:
public:
    Vector3D p0;
    Vector3D COM;
    const Body *body; // dummy body to satisfy compilation of base classes. Never to be used
    double width;
    double mass;
    std::atomic<Body_children> body_children; // this is a pointer to an array  !

    Node()
    {

        width = 0;
        mass = 0;
        COM = Vector3D(0, 0, 0);
        body_children.store(Body_children(nullptr, nullptr), std::memory_order_release);
    }

    Node(Vector3D p, double node_width)
    {
        p0 = p;
        width = node_width;
        mass = 0;
        COM = Vector3D(0, 0, 0);

        body_children.store(Body_children(nullptr, nullptr), std::memory_order_release);
    }

    bool is_leaf() const
    {
        return body_children.load().children == nullptr; // Is atomic necessary here ?
    }

    void make_children()
    {
        // usually make_children call is followed by move

        const Body *old_body = body_children.load().b;
        if (old_body == nullptr)
        {
            // Someone else moved it, means children. are there
            return;
        }

        Vector3D old_b = old_body->pos;
        double half_width = width * 0.5;
        unsigned mov_idx =
            (unsigned)(old_b.x > p0.x + half_width) * 1 +
            (unsigned)(old_b.y > p0.y + half_width) * 2 +
            (unsigned)(old_b.z > p0.z + half_width) * 4;
        std::vector<Vector3D> p(8); // Requires default constructor

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

        Node *tmp_children = new Node[8]; // calls default constructor !
        for (unsigned i = 0; i < 8; ++i)
        {
            tmp_children[i].p0 = p[i];
            tmp_children[i].width = half_width;
        }
        Body_children child_state = Body_children(old_body, nullptr);
        tmp_children[mov_idx].body_children.store(child_state);

        if (body_children.compare_exchange_weak(child_state, Body_children(nullptr, tmp_children), std::memory_order_release))
        {
            return;
        }
        else
        {
            // Someone else got it
            delete[] tmp_children;
        }
    }

    Node *get_child(const int idx) const
    {
        return body_children.load().children + idx; // Guaranteed only for non nullptr
    }

    void move_body_with_mass(int mov_idx)
    {
        throw;
        // Someone might also be doing operations on children !
    }

    void move_body_ptrOnly(int mov_idx)
    {
        throw;
    }

    const Body *get_body() const
    {
        // TO be used only in force compute !
        return body_children.load(std::memory_order_relaxed).b;
    }

    void store_body(const Body *new_body)
    {
        // Only to be used in reorder step !
        Body_children cur = body_children.load();
        cur.b = new_body;
        body_children.store(cur, std::memory_order_relaxed);
    }

    ~Node()
    {

        delete[] body_children.load().children;
    }
};
