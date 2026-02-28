#include "baseline.h"
#include "utils.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

void MultiThreadedForceSim::compute_acceleration_thread(const std::vector<Body> &Bodies, const Node *const root, const int start, const int end, std::vector<Vector3D> &accelerations, unsigned i1)
{
    uint bi = start;
    for (bi = start; bi + BLOCK_SIZE < end; bi += BLOCK_SIZE)
    {
        compute_acceleration_block(&Bodies[bi], root, &accelerations[i1 + bi]);
    }

    // clean up code

    for (bi = bi; bi < end; bi++)
    {
        accelerations[i1 + bi] = compute_acceleration(&Bodies[bi], root, theta);
    }
}

void MultiThreadedForceSim::compute_acceleration_all(const std::vector<Body> &Bodies, const Node *const root, std::vector<Vector3D> &accelerations, unsigned i1)
{
    std::vector<std::thread> workers;
    const int N = Bodies.size();
    const int num_bodies_pert = N / nThreads;

    for (int i = 0; i < nThreads; i++)
    {

        workers.push_back(std::thread(&MultiThreadedForceSim::compute_acceleration_thread, this, ref(Bodies), root, i * num_bodies_pert, std::min(N, (i + 1) * num_bodies_pert), ref(accelerations), i1));
    }

    for (auto &t : workers)
    {
        t.join();
    }

    // cleanup
    for (int bi = std::min(N, nThreads * num_bodies_pert); bi < N; bi++)
    {
        accelerations[i1 + bi] = compute_acceleration(&Bodies[bi], root, theta);
    }
}
