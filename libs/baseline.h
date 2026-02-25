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
    void leapFrog(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1);

protected:
    const double dt;
    const double theta;
    virtual Node *construct_tree(const std::vector<Body> &Bodies);
    virtual void insert(const Body *const b, Node *const n);
    virtual void reorder(Node *const root, std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1);
    virtual Vector3D compute_acceleration(const Body *const b, const Node *const node, const double theta); // Why extra parameter theta ?
    virtual void compute_acceleration_all(const std::vector<Body> &Bodies, const Node *const root, std::vector<Vector3D> &accelerations, unsigned i1);
    virtual Statistics timestep(std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1);

public:
    Statistics run(std::vector<Body> &Bodies, const int num_steps);
    BaseSim(double dt, double theta) : dt(dt), theta(theta) {}
};

class PostCOMSim : public BaseSim
{
private:
    void compute_COM_post(Node *const n);
    void insert_COM_post(const Body *const b, Node *const n);

protected:
    Node *construct_tree(const std::vector<Body> &Bodies) override;

public:
    PostCOMSim(double dt, double theta) : BaseSim(dt, theta) {}
};

class IterativeSim : public BaseSim
{
    // Construct tree logic is the same as base
    // timestep logic is the same
    // run logic is same as base
    // insert and compute acceleration logic is different.

protected:
    Vector3D compute_acceleration(const Body *const b, const Node *const node, const double theta) override;
    void insert(const Body *const b, Node *const n) override; // Virtual call overhead, but we get away with it due to iterative style

public:
    IterativeSim(double dt, double theta) : BaseSim(dt, theta) {}
};

class DFSOrderSim : public IterativeSim
{

    // Construct tree logic is same
    // Run logic is same
    // timstep logic is same
    // Compute acceleration is same
    // Reordering logic is different
private:
protected:
    void reorder(Node *const root, std::vector<Body> &Bodies, std::vector<Vector3D> &accelerations, unsigned i1) override;

public:
    DFSOrderSim(double dt, double theta) : IterativeSim(dt, theta) {}
};

// Keeping Block size as compile time constant
#ifndef BLOCK_SIZE
#define BLOCK_SIZE 4
#endif

class BodyBlockingSim : public DFSOrderSim // Needs reordering for Body Blocking
{
    // compute_acceleration_all is different, have to handle Blocked compute
private:
    uint total_loops{0};
    uint adv{0};

protected:
    virtual void compute_acceleration_block(const Body *bodies, const Node *const root, Vector3D *accelerations);

    void compute_acceleration_all(const std::vector<Body> &Bodies, const Node *const root, std::vector<Vector3D> &accelerations, unsigned i1) override;

public:
    BodyBlockingSim(double dt, double theta) : DFSOrderSim(dt, theta) {}
};

class AVXSim : public BodyBlockingSim
{
    // Only compute acceleration block logic is different. Valid only for BLOCK_SIZE 4
protected:
    virtual void compute_acceleration_block(const Body *bodies, const Node *const root, Vector3D *accelerations) override;

public:
    AVXSim(double dt, double theta) : BodyBlockingSim(dt, theta) {}
};
