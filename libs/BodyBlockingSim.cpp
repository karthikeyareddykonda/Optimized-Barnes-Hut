#include "baseline.h"
#include "utils.h"

void BodyBlockingSim::compute_acceleration_block(const Body *bodies, const Node *const root, Vector3D *accelerations)
{
    std::vector<std::pair<const Node *, int>> rec_stack;
    rec_stack.push_back(std::make_pair(root, (1 << BLOCK_SIZE) - 1));

    // Warning algorithm guaranteed only for num_bodies >= 2 and theta <= 1
    while (!rec_stack.empty())
    {
        total_loops++;

        const Node *const parent = rec_stack.back().first;
        int mask = rec_stack.back().second;
        rec_stack.pop_back();
        if ((mask & (mask - 1)) != 0)
            adv++;
        //  std::cout << "node depth : " << parent->width << "\n";
        for (unsigned int i = 0; i < 8; i++)
        {
            int sub_mask = mask;
            const Node *node = parent->get_child(i); // Incorrect semantics. parent itself maybe a leaf!
                                                     //  std::cout << "child : " << node << "\n";
            for (uint bind = 0; bind < BLOCK_SIZE; bind++)
            {
                const Body *b = bodies + bind;
                bool eval_body = (((mask >> bind) & 1) == 1);
                if (!eval_body)
                    continue;
                if (node->is_leaf())
                { // no children -> end node
                    // No need to bother about submask computation here
                    if (node->body == NULL || node->body == b)
                    { // no attached body -> zero acceleration
                        continue;
                    }

                    // has attached body -> compute acceleration between the two bodies
                    Vector3D b_p = b->pos;
                    Vector3D COM = node->COM;
                    Vector3D r{b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
                    double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
                    double fac = -G * node->body->mass / (r_norm * r_norm * r_norm);
                    Vector3D contrib{fac * r.x, fac * r.y, fac * r.z};
                    *(accelerations + bind) += contrib;
                    continue;
                }

                // not end node -> check if width/distance < theta
                Vector3D b_p = b->pos;
                Vector3D COM = node->COM;
                Vector3D r{b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
                double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
                if (node->width / r_norm < theta)
                { // less than theta -> compute acceleration between node and body
                    // body approximated out
                    sub_mask ^= (1 << bind);
                    double fac = -G * node->mass / (r_norm * r_norm * r_norm);
                    Vector3D contrib{fac * r.x, fac * r.y, fac * r.z};
                    *(accelerations + bind) += contrib;
                    continue;
                }
            }

            // Push the child to stack
            if (sub_mask != 0 && !node->is_leaf())
            {

                rec_stack.push_back(std::make_pair(node, sub_mask));
            }
        }
    }
}
void BodyBlockingSim::compute_acceleration_all(const std::vector<Body> &Bodies, const Node *const root, std::vector<Vector3D> &accelerations, unsigned i1)
{
    const int N = Bodies.size();
    unsigned i0 = 0;
    if (i1 == 0)
        i0 = N;

    uint bi = 0;
    // std::cout << "reached \n";
    for (bi; bi + BLOCK_SIZE < N; bi += BLOCK_SIZE)
    {
        compute_acceleration_block(&Bodies[bi], root, &accelerations[i1 + bi]);
        //   std::cout << "block done \n";
    }
    // std::cout << "block done \n";
    // clean up code. Singular compute

    for (bi; bi < N; bi++)
    {
        accelerations[i1 + bi] = compute_acceleration(&Bodies[bi], root, theta);
    }

    std::cout << "reuse ratio : " << adv * 100.0 / total_loops << "\n";
}
