#include "baseline.h"
#include "utils.h"

void DFSOrderSim::reorder(Node *const root, std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1)
{
    // Warning ! :Gonna reseat the Bodies and accelerations array !
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

    // Reseating the data
    Bodies = std::move(Bodies2);
    accelerations = std::move(accelerations2);
}
