#include "baseline.h"
#include "utils.h"
#include "math.h"
#include <thread>

#include "Node_atomic.h"

void LockFreeSim::insert(const Body *const b, Node *const n)
{
    // Iterative version of tree construction !
    // All write operations happen atomically !
    Node *node = n;
    Vector3D new_b = b->pos;
    while (true)
    {

        Body_children expected(nullptr, nullptr);
        if (node->body_children.compare_exchange_weak(expected, Body_children(b, nullptr)))
        {
            // A node with null body and null children now has body and null children !
            // Succesful insert
            return; // bail out
        }

        // Expected is loaded with children pointer
        // Needs strict memory order !
        if (expected.children != nullptr)
        {
            // Move on to children logic
            Vector3D p0 = node->p0;
            double half_width = node->width * 0.5;
            unsigned new_idx =
                (unsigned)(new_b.x > p0.x + half_width) * 1 +
                (unsigned)(new_b.y > p0.y + half_width) * 2 +
                (unsigned)(new_b.z > p0.z + half_width) * 4;

            node = node->get_child(new_idx);
        }
        else
        {
            // No children  but body case !
            node->make_children(); // Has CAS inside it, moves body if it has some

            Vector3D p0 = node->p0;
            double half_width = node->width * 0.5;

            unsigned new_idx =
                (unsigned)(new_b.x > p0.x + half_width) * 1 +
                (unsigned)(new_b.y > p0.y + half_width) * 2 +
                (unsigned)(new_b.z > p0.z + half_width) * 4;
            node = node->get_child(new_idx);
        }

        // traverse tree until an end node is reached

        // node->w_lock.unlock(); // Safe to to release here
    }
}

Vector3D LockFreeSim::compute_acceleration(const Body *const b, const Node *const root, const double theta)
{
    std::vector<const Node *> rec_stack;
    rec_stack.push_back(root);
    Vector3D res(0, 0, 0);
    // Warning algorithm guaranteed only for num_bodies >= 2 and theta <= 1
    while (!rec_stack.empty())
    {

        const Node *const parent = rec_stack.back();
        rec_stack.pop_back();
        for (unsigned int i = 0; i < 8; i++)
        {
            const Node *node = parent->get_child(i);
            const Body *node_body = node->get_body();
            if (node->is_leaf())
            { // no children -> end node
                if (node_body == NULL || node_body == b)
                { // no attached body -> zero acceleration
                    continue;
                }

                // has attached body -> compute acceleration between the two bodies
                Vector3D b_p = b->pos;
                Vector3D COM = node->COM;
                Vector3D r{b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
                double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
                double fac = -G * node_body->mass / (r_norm * r_norm * r_norm);
                Vector3D contrib{fac * r.x, fac * r.y, fac * r.z};
                res += contrib;
                continue;
            }

            // not end node -> check if width/distance < theta
            Vector3D b_p = b->pos;
            Vector3D COM = node->COM;
            Vector3D r{b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
            double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
            if (node->width / r_norm < theta)
            { // less than theta -> compute acceleration between node and body
                double fac = -G * node->mass / (r_norm * r_norm * r_norm);
                Vector3D contrib{fac * r.x, fac * r.y, fac * r.z};
                res += contrib;
                continue;
            }

            // Push the child to stack

            rec_stack.push_back(node);
        }
    }

    return res;
}

void LockFreeSim::compute_COM_post(Node *const n)
{
    Node *node = n;
    if (node->is_leaf())
    {
        const Body *node_body = node->get_body();
        if (node_body != nullptr)
        {
            n->COM = node_body->pos;
            n->mass = node_body->mass;
        }
        // Default constructor init values are COM(0,0,0), and mass = 0
        return;
    }

    Vector3D COM(0, 0, 0);
    double tot_mass = 0;

    for (unsigned i = 0; i < 8; i++)
    {
        Node *const child = node->get_child(i);
        compute_COM_post(child);
        Vector3D childCOM = child->COM;
        double child_mass = child->mass;

        COM.x += child_mass * childCOM.x;
        COM.y += child_mass * childCOM.y;
        COM.z += child_mass * childCOM.z;
        tot_mass += child_mass;
    }
    double inv_mass = 1.0 / tot_mass;
    node->COM.x = COM.x * inv_mass;
    node->COM.y = COM.y * inv_mass;
    node->COM.z = COM.z * inv_mass;
    node->mass = tot_mass;
}

void LockFreeSim::construct_tree_thread(Node *const root, const std::vector<Body> &Bodies, const int start, const int end)
{
    // Unit of work per thread

    for (int i = start; i < end; i++)
    {
        insert(&Bodies[i], root);
    }
}

Node *LockFreeSim::construct_tree(const std::vector<Body> &Bodies)
{
    // Warning , there's entire tree allocated here. Have to be carefully destroyed before calling construct again
    auto limits = find_min_max(Bodies);
    double width = limits.second - limits.first;
    Vector3D p0_root{limits.first - 1.5 * width, limits.first - 1.5 * width, limits.first - 1.5 * width};

    Node *root = new Node(p0_root, 4 * width);

    const int N = Bodies.size();
    const int num_bodies_pert = N / nThreads;
    // std::cout << "num_pert " << num_bodies_pert << "\n";
    std::vector<std::thread> workers;

    for (int i = 0; i < nThreads; i++)
    {
        workers.push_back(std::thread(&LockFreeSim::construct_tree_thread, this, root, ref(Bodies), i * num_bodies_pert, std::min(N, (i + 1) * num_bodies_pert)));
    }

    // std::cout << "End index: " << i << "\n";

    for (auto &t : workers)
    {
        t.join();
    }

    // clean up
    for (int bi = std::min(N, nThreads * num_bodies_pert); bi < N; bi++)
    {
        insert(&Bodies[bi], root);
    }

    compute_COM_post(root);
    return root;
}

void LockFreeSim::reorder(Node *const root, std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1)
{
    // Warning ! :Gonna reseat the Bodies and accelerations array !
    /*
    const int N = Bodies.size();
    std::vector<Body> Bodies2;
    Bodies2.reserve(N); // Guaranteed to have size N, default constructor not defined !
    std::vector<Vector3D> accelerations2(2 * N);
    unsigned i0 = 0;
    if (i1 == 0)
        i0 = N;

    // Perform DFS and rewrite the body pointers in the tree, no longer a const Node ptr
    std::vector<Node *> rec_stack;
    rec_stack.push_back(root);

    int new_array_off = 0;
    while (!rec_stack.empty())
    {
        Node *node = rec_stack.back();
        rec_stack.pop_back();
        const Body *node_body = node->get_body();
        if (node->is_leaf() && node_body != nullptr)
        {

            Bodies2.push_back(*(node_body)); // Deep copy
            int old_array_off = (node_body) - &Bodies[0];
            accelerations2[i0 + new_array_off] = accelerations[i0 + old_array_off];
            node->store_body(&Bodies2.back());
            new_array_off++;
        }
        else if (!node->is_leaf())
        {
            for (uint i = 0; i < 8; i++)
            {
                rec_stack.push_back(node->get_child(i));
            }
        }
    }

    // Post COM compute needs botttom up recursion!
    // Can try collecting all Node in a vector and iterate in reverse
    // Children gets computed first , then parent guaranteed
    // Can parallelise it with sync

    // For now we'll use vanilla post COM compute

    // compute_COM_post(root);

    // Reseating the data
    Bodies = std::move(Bodies2);
    accelerations = std::move(accelerations2);
    */
}
