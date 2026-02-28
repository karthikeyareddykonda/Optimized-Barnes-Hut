#include "baseline.h"
#include "utils.h"
#include "math.h"
#include "Node_mutex.h"
#include <thread>

void MultiThreadedSim::reorder(Node *const root, std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1)
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
        if (node->is_leaf() && node->body != nullptr)
        {

            Bodies2.push_back(*(node->body)); // Deep copy
            int old_array_off = (node->body) - &Bodies[0];
            accelerations2[i0 + new_array_off] = accelerations[i0 + old_array_off];
            node->body = &Bodies2.back();
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

    compute_COM_post(root);

    // Reseating the data
    Bodies = std::move(Bodies2);
    accelerations = std::move(accelerations2);
    */
}

void MultiThreadedSim::compute_COM_post(Node *const n)
{
    // No need to acquire locks here. It is single threaded only
    Node *node = n;
    if (node->is_leaf())
    {
        if (node->body != nullptr)
        {
            n->COM = n->body->pos;
            n->mass = n->body->mass;
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

void MultiThreadedSim::insert(const Body *const b, Node *const n)
{
    // Iterative version of tree construction !
    // All write operations happen atomically !
    Node *node = n;
    Vector3D new_b = b->pos;
    while (true)
    {
        if (!node->is_leaf())
        {
            // No need locks here
            Vector3D p0 = node->p0;
            double half_width = node->width * 0.5;
            unsigned new_idx =
                (unsigned)(new_b.x > p0.x + half_width) * 1 +
                (unsigned)(new_b.y > p0.y + half_width) * 2 +
                (unsigned)(new_b.z > p0.z + half_width) * 4;

            node = node->get_child(new_idx);
            continue;
        }
        else
        { // no children -> end node
            node->w_lock.lock();
            if (node->is_leaf())
            {
                if (node->body == nullptr)
                { // no attached body -> free for insert
                    node->body = b;
                    node->w_lock.unlock();
                    return;
                }

                // has attached body -> make children, move attached body and new body into children
                node->make_children();
                Vector3D old_b = node->body->pos;

                Vector3D p0 = node->p0;
                double half_width = node->width * 0.5;
                unsigned mov_idx =
                    (unsigned)(old_b.x > p0.x + half_width) * 1 +
                    (unsigned)(old_b.y > p0.y + half_width) * 2 +
                    (unsigned)(old_b.z > p0.z + half_width) * 4;
                unsigned new_idx =
                    (unsigned)(new_b.x > p0.x + half_width) * 1 +
                    (unsigned)(new_b.y > p0.y + half_width) * 2 +
                    (unsigned)(new_b.z > p0.z + half_width) * 4;

                node->move_body_ptrOnly(mov_idx);
                node->w_lock.unlock(); // Release parent lock first

                node = node->get_child(new_idx);

                continue;
            }
            node->w_lock.unlock();

            // Again finding not a leaf !
            // retry !
        }

        // traverse tree until an end node is reached

        // node->w_lock.unlock(); // Safe to to release here
    }
}

void MultiThreadedSim::construct_tree_thread(Node *const root, const std::vector<Body> &Bodies, const int start, const int end)
{
    // Unit of work per thread

    for (int i = start; i < end; i++)
    {
        insert(&Bodies[i], root);
    }
}

Node *MultiThreadedSim::construct_tree(const std::vector<Body> &Bodies)
{
    // Warning , there's entire tree allocated here. Have to be carefully destroyed before calling construct again
    auto limits = find_min_max(Bodies);
    double width = limits.second - limits.first;
    Vector3D p0_root{limits.first - 1.5 * width, limits.first - 1.5 * width, limits.first - 1.5 * width};

    Node *root = new Node(p0_root, 4 * width);

    const int N = Bodies.size();
    const int num_bodies_pert = N / nThreads;

    std::vector<std::thread> workers;
    for (int i = 0; i < nThreads; i++)
    {
        workers.push_back(std::thread(&MultiThreadedSim::construct_tree_thread, this, root, ref(Bodies), i * num_bodies_pert, std::min(N, (i + 1) * num_bodies_pert)));
    }

    for (auto &t : workers)
    {
        t.join();
    }

    // clean up
    for (int bi = std::min(N, nThreads * num_bodies_pert); bi < N; bi++)
    {
        insert(&Bodies[bi], root);
    }

    return root;
}
