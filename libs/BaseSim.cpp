#include "baseline.h"
#include "utils.h"
#include <iostream>
#include <cmath>
#include <chrono>

void BaseSim::insert(const Body *const b, Node *const n)
{
    Node *node = n;
    if (node->is_leaf())
    { // no children -> end node
        if (node->body == nullptr)
        { // no attached body -> free for insert
            node->body = b;
            node->COM = b->pos;
            node->mass = b->mass;
            return;
        }

        // has attached body -> make children, move attached body and new body into children
        node->make_children();
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
        node->move_body_with_mass(mov_idx);

        insert(b, node->get_child(new_idx));
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
    Node *next_node = node->get_child(new_idx);

    insert(b, next_node);
}

Node *BaseSim::construct_tree(const std::vector<Body> &Bodies)
{
    // Warning , there's entire tree allocated here. Have to be carefully destroyed before calling construct again
    auto limits = find_min_max(Bodies);
    double width = limits.second - limits.first;
    Vector3D p0_root{limits.first - 1.5 * width, limits.first - 1.5 * width, limits.first - 1.5 * width};

    Node *root = new Node(p0_root, 4 * width);

    for (const auto &b : Bodies)
    {
        insert(&b, root); // Multiple new calls. here !
    }
    return root;
}

void BaseSim::reorder(Node *const root, std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1)
{
    // BaseSim doesn't do any reordering
}

Vector3D BaseSim::compute_acceleration(const Body *const body, const Node *const node, const double theta)
{
    if (node->is_leaf())
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
    Vector3D a0 = compute_acceleration(body, node->get_child(0), theta);
    Vector3D a1 = compute_acceleration(body, node->get_child(1), theta);
    Vector3D a2 = compute_acceleration(body, node->get_child(2), theta);
    Vector3D a3 = compute_acceleration(body, node->get_child(3), theta);
    Vector3D a4 = compute_acceleration(body, node->get_child(4), theta);
    Vector3D a5 = compute_acceleration(body, node->get_child(5), theta);
    Vector3D a6 = compute_acceleration(body, node->get_child(6), theta);
    Vector3D a7 = compute_acceleration(body, node->get_child(7), theta);
    Vector3D res = {
        a0.x + a1.x + a2.x + a3.x + a4.x + a5.x + a6.x + a7.x,
        a0.y + a1.y + a2.y + a3.y + a4.y + a5.y + a6.y + a7.y,
        a0.z + a1.z + a2.z + a3.z + a4.z + a5.z + a6.z + a7.z};
    return res;
}

void BaseSim::compute_acceleration_all(const std::vector<Body> &Bodies, const Node *const root, std::vector<Vector3D> &accelerations, unsigned i1)
{
    const unsigned int N = Bodies.size();
    unsigned i0 = 0;
    if (i1 == 0)
        i0 = N;
    for (unsigned i = 0; i < N; ++i)
    {
        accelerations[i1 + i] = compute_acceleration(&Bodies[i], root, theta);
    }
}

void BaseSim::leapFrog(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1)
{
    const unsigned int N = Bodies.size();
    unsigned i0 = 0;
    if (i1 == 0)
        i0 = N;
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
}
Statistics BaseSim::timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1)
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
    Node *root = construct_tree(Bodies);
    auto end_insert = std::chrono::high_resolution_clock::now();
    times.t_insert += std::chrono::duration<double>(end_insert - start_insert).count();

    auto start_reorder = std::chrono::high_resolution_clock::now();
    reorder(root, Bodies, accelerations, i1);
    auto end_reorder = std::chrono::high_resolution_clock::now();
    times.t_reorder += std::chrono::duration<double>(end_reorder - start_reorder).count();

    // force calculation

    auto start_force = std::chrono::high_resolution_clock::now();
    compute_acceleration_all(Bodies, root, accelerations, i1);
    auto end_force = std::chrono::high_resolution_clock::now();

    times.t_force += std::chrono::duration<double>(end_force - start_force).count();

    // leapfrog integration
    auto start_leapfrog = std::chrono::high_resolution_clock::now();
    leapFrog(Bodies, accelerations, i1);
    auto end_leapfrog = std::chrono::high_resolution_clock::now();
    times.t_leapfrog += std::chrono::duration<double>(end_leapfrog - start_leapfrog).count();

    // tree deletion
    // we delete all the children only. Root is intact
    // Also counted in tree construction
    start_insert = std::chrono::high_resolution_clock::now();
    delete root;
    end_insert = std::chrono::high_resolution_clock::now();
    times.t_insert += std::chrono::duration<double>(end_insert - start_insert).count();

    return times;
}

Statistics BaseSim::run(std::vector<Body> &Bodies, const int num_steps)
{

    Statistics times;
    const int N = Bodies.size();
    std::vector<Vector3D> Accelerations(2 * N);
    for (uint i = 0; i < num_steps; i++)
    {
        Statistics times_per_step = timestep(Bodies, Accelerations, (i % 2) * N); // Choice : Use a class of parameters. dt, theta etc
        times.t_insert += times_per_step.t_insert;
        times.t_force += times_per_step.t_force;
        times.t_leapfrog += times_per_step.t_leapfrog;
        times.t_reorder += times_per_step.t_reorder;
    }
    // Add reorder overhead in construction for brevity reporting
    // times.t_insert += times.t_reorder;
    return times;
}
