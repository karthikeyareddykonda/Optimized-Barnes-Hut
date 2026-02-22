#pragma once
#include <vector>
#include <iostream>
#include "common.h"

#ifdef USE_CONTIGUOUS
#include "Node_contig.h"
#else
#include "Node.h"
#endif

class BaseSim
{
private:
    const double dt;
    const double theta;

    Vector3D compute_acceleration(const Body *const b, const Node *const n, const double theta);
    Statistics timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1);
    void insert(const Body *const b, Node *const n);

protected:
    virtual Node *construct_tree(const std::vector<Body> &Bodies);

public:
    Statistics run(std::vector<Body> &Bodies, const int num_steps);
    BaseSim(double dt, double theta) : dt(dt), theta(theta) {}
};

class PostCOMSim : public BaseSim
{
private:
    void compute_COM_post(Node *const n);
    void insert(const Body *const b, Node *const n);

protected:
    Node *construct_tree(const std::vector<Body> &Bodies) override;

public:
    PostCOMSim(double dt, double theta) : BaseSim(dt, theta) {}
};
