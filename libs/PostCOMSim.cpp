#include "baseline.h"
#include "utils.h"

void PostCOMSim::insert_COM_post(const Body *const b, Node *const n)
{
    // this insert doesn't do any COM related compute
    Node *node = n;
    if (node->is_leaf())
    { // no children -> end node
        if (node->body == nullptr)
        { // no attached body -> free for insert
            node->body = b;
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

        node->move_body_ptrOnly(mov_idx);

        insert_COM_post(b, node->get_child(new_idx));
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

    Node *next_node = node->get_child(new_idx);

    insert_COM_post(b, next_node);
}

void PostCOMSim::compute_COM_post(Node *const n)
{
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

Node *PostCOMSim::construct_tree(const std::vector<Body> &Bodies)
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

    compute_COM_post(root);
    return root;
}
