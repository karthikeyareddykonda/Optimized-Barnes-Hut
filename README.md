# Optimized Barnes-Hut N-Body Simulation

An efficient, high-performance implementation of the Barnes-Hut algorithm for $N$-body gravitational simulations. This project tracks the evolution from a baseline recursive implementation to a highly optimized, hardware-aware system.

## The Origin: Barnes & Hut (1986)

The Barnes-Hut algorithm was first introduced by **Josh Barnes** and **Piet Hut** in their 1986 *Nature* paper, *"A hierarchical $O(N \log N)$ force-calculation algorithm"*. 

Before this breakthrough, simulating galactic-scale systems was computationally prohibitive. A "Brute Force" or Direct Summation approach requires calculating interactions between every pair of particles, resulting in **$O(N^2)$** complexity. For a simulation of 1 million particles, this would require **1 trillion** calculations per time step.



## The Complexity Shift

The core innovation of Barnes-Hut is the use of a hierarchical spatial data structure (a **Quadtree** in 2D or **Octree** in 3D) to group distant particles. By treating a cluster of distant bodies as a single point mass located at their collective center of mass, the algorithm reduces the number of interactions per particle from $N$ to $\log N$.

| Algorithm | Complexity | Operations for $N=10^6$ |
| :--- | :--- | :--- |
| **Brute Force** | $O(N^2)$ | $1,000,000,000,000$ |
| **Barnes-Hut** | $O(N \log N)$ | $\approx 20,000,000$ |

The balance between speed and physical accuracy is governed by the **Multipole Acceptance Criterion (MAC)**, controlled by the "opening angle" parameter $\theta$:
$$\frac{s}{d} < \theta$$
Where $s$ is the width of the node and $d$ is the distance to the particle.



## Core Phases

Every time step in the simulation follows a two-phase cycle:

1. **Tree Construction:** Recursively partition the simulation space into a tree until every particle resides in its own leaf node, then compute the mass and center of mass for every internal node.
2. **Force Calculation:** For each particle, traverse the tree. Use the aggregate mass of a node if it satisfies the $\theta$ criterion; otherwise, recurse into its children for a higher-resolution calculation.
