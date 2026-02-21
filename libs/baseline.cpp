#include "baseline.h"
#include <iostream>

static inline void make_children(Node *n)
{

    std::vector<Vector3D> p(8);
    double half_width = n->width * 0.5;
    Vector3D p0 = n->p0; // Choice. Value deep copy, is it required ?
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
        n->children[i] = new Node();
        n->children[i]->width = half_width;
        n->children[i]->p0 = p[i];
    }
}

void insert(const Body *const b, Node *const n)
{
    Node *node = n;
    if (node->children[0] == nullptr)
    { // no children -> end node
        if (node->body == nullptr)
        { // no attached body -> free for insert
            node->body = b;
            node->COM = b->pos;
            node->mass = b->mass;
            return;
        }

        // has attached body -> make children, move attached body and new body into children
        make_children(node);
        Vector3D old_b = node->body->pos; // choice : deep copy //to_cartesian(&node->body->p);
        Vector3D new_b = b->pos;          // to_cartesian(&b->p);
        Vector3D p0 = node->p0;           // to_cartesian(&node->p0);
        double half_width = node->width * 0.5;
        unsigned mov_idx =
            (unsigned)(old_b.x > p0.x + half_width) * 1 +
            (unsigned)(old_b.y > p0.y + half_width) * 2 +
            (unsigned)(old_b.z > p0.z + half_width) * 4;
        unsigned new_idx =
            (unsigned)(new_b.x > p0.x + half_width) * 1 +
            (unsigned)(new_b.y > p0.y + half_width) * 2 +
            (unsigned)(new_b.z > p0.z + half_width) * 4;

        Vector3D COM = node->COM; // Deep copy !
        double inv_mass = 1.0 / (node->mass + b->mass);
        COM.x = (COM.x * node->mass + new_b.x * b->mass) * inv_mass;
        COM.y = (COM.y * node->mass + new_b.y * b->mass) * inv_mass;
        COM.z = (COM.z * node->mass + new_b.z * b->mass) * inv_mass;

        node->mass = node->mass + b->mass;
        node->COM = COM; // Deep copy !
        node->children[mov_idx]->body = node->body;
        node->children[mov_idx]->COM = node->body->pos;
        node->children[mov_idx]->mass = node->body->mass;
        node->body = nullptr;

        insert(b, node->children[new_idx]);
        return;
    }

    // traverse tree until an end node is reached

    Vector3D new_b = b->pos;
    Vector3D p0 = node->p0;
    double half_width = node->width * 0.5;
    unsigned new_idx =
        (unsigned)(new_b.x > p0.x + half_width) * 1 +
        (unsigned)(new_b.y > p0.y + half_width) * 2 +
        (unsigned)(new_b.z > p0.z + half_width) * 4;

    // COM adjust
    Vector3D COM = node->COM;
    double inv_mass = 1.0 / (node->mass + b->mass);
    COM.x = (COM.x * node->mass + new_b.x * b->mass) * inv_mass;
    COM.y = (COM.y * node->mass + new_b.y * b->mass) * inv_mass;
    COM.z = (COM.z * node->mass + new_b.z * b->mass) * inv_mass;

    node->mass = node->mass + b->mass;
    node->COM = COM; // Seems redundant
    Node *next_node = node->children[new_idx];

    insert(b, next_node);
}

Vector3D compute_acceleration(const Body *const body, const Node *const node, const double theta)
{
    if (node->children[0] == NULL)
    { // no children -> end node
        if (node->body == NULL || node->body == body)
        { // no attached body -> zero acceleration
            Vector3D res = {0.0, 0.0, 0.0};
            return res;
        }

        // has attached body -> compute acceleration between the two bodies
        Vector3D b_p = body->pos;
        Vector3D COM = node->COM;
        Vector3D r = {b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
        double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
        double fac = -G * node->body->mass / (r_norm * r_norm * r_norm);
        Vector3D res = {fac * r.x, fac * r.y, fac * r.z};
        return res;
    }

    // not end node -> check if width/distance < theta
    Vector3D b_p = body->pos;
    Vector3D COM = node->COM;
    Vector3D r = {b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
    double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    if (node->width / r_norm < theta)
    { // less than theta -> compute acceleration between node and body
        double fac = -G * node->mass / (r_norm * r_norm * r_norm);
        Vector3D res = {fac * r.x, fac * r.y, fac * r.z};
        return res;
    }

    // greater than theta -> sum up accelerations from child nodes
    Vector3D a0 = compute_acceleration(body, node->children[0], theta);
    Vector3D a1 = compute_acceleration(body, node->children[1], theta);
    Vector3D a2 = compute_acceleration(body, node->children[2], theta);
    Vector3D a3 = compute_acceleration(body, node->children[3], theta);
    Vector3D a4 = compute_acceleration(body, node->children[4], theta);
    Vector3D a5 = compute_acceleration(body, node->children[5], theta);
    Vector3D a6 = compute_acceleration(body, node->children[6], theta);
    Vector3D a7 = compute_acceleration(body, node->children[7], theta);
    Vector3D res = {
        a0.x + a1.x + a2.x + a3.x + a4.x + a5.x + a6.x + a7.x,
        a0.y + a1.y + a2.y + a3.y + a4.y + a5.y + a6.y + a7.y,
        a0.z + a1.z + a2.z + a3.z + a4.z + a5.z + a6.z + a7.z};
    return res;
}

Statistics timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, Node *root, const double dt, const double theta, unsigned i1)
{
    // Each thread better use it's own statistics, to avoid race even for statistic collection
    Statistics times;

    // determine whether the first half of the acceleration array is old or the second half
    const unsigned int N = Bodies.size();
    unsigned i0 = 0;
    if (i1 == 0)
        i0 = N;

    // insert bodies into the tree
    auto start_insert = std::chrono::high_resolution_clock::now();
    for (const auto &b : Bodies)
    {
        insert(&b, root);
    }
    auto end_insert = std::chrono::high_resolution_clock::now();

    times.t_insert += std::chrono::duration<double>(end_insert - start_insert).count();

    // force calculation

    auto start_force = std::chrono::high_resolution_clock::now();

    for (unsigned i = 0; i < N; ++i)
    {
        accelerations[i1 + i] = compute_acceleration(&Bodies[i], root, theta);
    }

    auto end_force = std::chrono::high_resolution_clock::now();

    times.t_force += std::chrono::duration<double>(end_force - start_force).count();

    // leapfrog integration
    auto start_leapfrog = std::chrono::high_resolution_clock::now();
    for (unsigned i = 0; i < N; ++i)
    {
        Vector3D p_car = Bodies[i].pos;
        p_car.x += (Bodies[i].vel.x + 0.5 * accelerations[i0 + i].x * dt) * dt;
        p_car.y += (Bodies[i].vel.y + 0.5 * accelerations[i0 + i].y * dt) * dt;
        p_car.z += (Bodies[i].vel.z + 0.5 * accelerations[i0 + i].z * dt) * dt;
        Bodies[i].pos = p_car;

        Bodies[i].vel.x += 0.5 * (accelerations[i0 + i].x + accelerations[i1 + i].x) * dt;
        Bodies[i].vel.y += 0.5 * (accelerations[i0 + i].y + accelerations[i1 + i].y) * dt;
        Bodies[i].vel.z += 0.5 * (accelerations[i0 + i].z + accelerations[i1 + i].z) * dt;
    }
    auto end_leapfrog = std::chrono::high_resolution_clock::now();
    times.t_leapfrog += std::chrono::duration<double>(end_leapfrog - start_leapfrog).count();

    // tree deletion
    // we delete all the children only. Root is intact
    // Also counted in tree construction
    start_insert = std::chrono::high_resolution_clock::now();
    root->reset_if_root();
    end_insert = std::chrono::high_resolution_clock::now();
    times.t_insert += std::chrono::duration<double>(end_insert - start_insert).count();

    return times;
}
