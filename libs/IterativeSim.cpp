#include "baseline.h"
#include "utils.h"
#include "math.h"

void IterativeSim::insert(const Body *const b, Node *const n)
{
    // Iterative version of tree construction !
    Node *node = n;
    Vector3D new_b = b->pos;
    while (true)
    {
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

            Vector3D COM = node->COM; // Deep copy !
            double inv_mass = 1.0 / (node->mass + b->mass);
            COM.x = (COM.x * node->mass + new_b.x * b->mass) * inv_mass;
            COM.y = (COM.y * node->mass + new_b.y * b->mass) * inv_mass;
            COM.z = (COM.z * node->mass + new_b.z * b->mass) * inv_mass;

            node->mass = node->mass + b->mass;
            node->COM = COM; // Deep copy !
            node->move_body_with_mass(mov_idx);

            node = node->get_child(new_idx);
            continue;
        }

        // traverse tree until an end node is reached

        Vector3D p0 = node->p0;
        double half_width = node->width * 0.5;
        unsigned new_idx =
            (unsigned)(new_b.x > p0.x + half_width) * 1 +
            (unsigned)(new_b.y > p0.y + half_width) * 2 +
            (unsigned)(new_b.z > p0.z + half_width) * 4;

        Vector3D COM = node->COM;
        double inv_mass = 1.0 / (node->mass + b->mass);
        COM.x = (COM.x * node->mass + new_b.x * b->mass) * inv_mass;
        COM.y = (COM.y * node->mass + new_b.y * b->mass) * inv_mass;
        COM.z = (COM.z * node->mass + new_b.z * b->mass) * inv_mass;

        node->mass = node->mass + b->mass;
        node->COM = COM; // Seems redundant
        node = node->get_child(new_idx);
    }
}

Vector3D IterativeSim::compute_acceleration(const Body *const b, const Node *const root, const double theta)
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
            if (node->is_leaf())
            { // no children -> end node
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
