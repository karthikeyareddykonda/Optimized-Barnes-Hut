# Optimized Barnes-Hut N-Body Simulation

#  Optimized Barnes-Hut: From Recursion to AVX
**Breaking the $O(N^2)$ Barrier with Hardware-Aware Engineering.**

An efficient, high-performance implementation of the Barnes-Hut algorithm for $N$-body gravitational simulations. This project tracks the evolution of a simulation engine from a baseline recursive implementation to a highly optimized system utilizing **Data-Oriented Design** and **SIMD vectorization**.

> [!IMPORTANT]
> **Key Result:** Achieved a **20x+ speedup** on a single thread compared to the GCC `-O3` compiled baseline.

---

### ⚡ Performance Highlights
| Version | Strategy | Speedup | Bottleneck Addressed |
| :--- | :--- | :--- | :--- |
| **1** | Baseline (GCC -O3) | $1.0\times$ | Algorithmic Complexity |
| **2** |  Layout | $1.8\times$ | Total Data footprint |
| **3** | Iterative| $2.8\times$ | Function Stack overhead |
| **4** | **Body reorder** | **9x** | Data movement in hardware |
| **5** | **Body blocking** | **11x** | Instruction Throughput (FLOPs) |
| **6** | **AVX2 SIMD** | **20x** | Instruction Throughput (FLOPs) |


* **Language:** C++17
* **Architecture:** x86_64 (AVX2, FMA)

---
## The Origin: Barnes & Hut (1986)

The Barnes-Hut algorithm was first introduced by **Josh Barnes** and **Piet Hut** in their 1986 *Nature* paper, *"A hierarchical O(N log N) force-calculation algorithm"*. 

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

### Challenges for performance
1. Memory Wall : Tree based methods involve pointer chasing. Difficult for hardware prefetchers, destroying cache locality
2. Branch Misprediction :  Recursive if-else logic checking the criteria
3. Floating-point square roots and divisions are expensive scalar operations


## 🧩 The Baseline: Recursive Barnes-Hut
The initial implementation follows the standard recursive depth-first traversal. While mathematically intuitive, this structure is the primary source of performance bottlenecks due to frequent branching and stack overhead.

### Logic: `ComputeAcceleration(Body b, Node n)`
```python
def compute_acceleration(body, node):
    dist = distance_between(body, node.center_of_mass)
    
    # Multipole Acceptance Criterion (MAC)
    if (node.is_leaf) or (node.width / dist < theta):
        # APPROXIMATE: Treat the entire node as a single point mass
        body.acceleration += calculate_force(node.mass, node.center_of_mass)
    else:
        # REFINE: Move deeper into the tree for higher precision
        for child in node.children:
            compute_acceleration(body, child)
```
### Logic: `InsertBody(Node n, Body b)`
```python
def insert_body(node, body):
    if node.is_empty:
        # Place body in this leaf
        node.body = body
    elif node.is_leaf:
        # Subdivide: Create 8 children and re-insert the existing body
        node.subdivide()
        old_body = node.body
        node.body = None
        
        insert_body(node, old_body)
        insert_body(node, body)
    else:
        # Internal node: Determine which octant the body belongs to
        octant = get_octant(node, body)
        insert_body(node.children[octant], body)
    
    # Update Center of Mass and total mass for this node
    update_mass_properties(node)
```
For $\theta =0.5$ usually the force compute composes of over 90% execution time

# Performance Overview




# Optimization Journey

 ###  Post COM compute  

The Idea: Postpone Center of Mass (COM) calculations until the tree is fully built to simplify the insertion logic.

The Result: Performance Degradation of tree construction by about 10 %

The Insight: While we reduced the number of floating-point operations during insertion, the requirement for a separate post-build tree traversal increased memory traffic and cache misses. The cost of "moving data twice" outweighed the mathematical savings, so this approach was discarded in favor of fused COM updates.

 ### Contiguous Tree Data Layout

The idea : Replace the std::vector<Node*> (which stores pointers to heap-allocated nodes spread across memory) with a single Node* children pointer that points to a contiguous block of 8 child nodes 

The Result: Substantial reduction in "Pointer Chasing" latency and a more compact memory footprint. A total speed up of 1.8x on 0.5M objects . The Node Object collectively own less memory by 7 pointers 

The Insight: By ensuring that all 8 children of a node are physically adjacent in memory, we maximize spatial locality. When the CPU fetches the first child, the hardware prefetcher  can automatically pulls the subsequent children into the L1/L2 cache before the code even requests them.

### Recursive to iterative

The baseline involves recursive calls , which involves a DFS like traversal on the tree, not exploiting the continuity we created among the child nodes before. We change the recursive implementation to iterative implementation with custom stack ( it's simply a vector < node * > ) and doing the traversal more like a BFS.

The Result: A total speed up of 2.8x (including previous optimization)

The Insight : Consecutive cache line access improves the performance.


###  Body reorder

The idea :  
At first it seems that force compute among bodies are completely independent, indeed they are theoretically.  But one can observe closer the bodies, closer the approximation decision they make in the tree traversal. If the bodies have same co-ordinates, then they are gonna make the exact same decision.  For ex : computing force of milky way galaxy on earth and force of milky way galaxy on the sun. It's very likely that both the traversal end up using the same tree nodes due to their proximity. For the CPU this may not be apparent, but in case if earth and sun are computed one after the other. It is very likely that the tree nodes brought into cache by the former are being used by the later. So we decided to test the idea. We have the choices.   

1. Add some extra clustering algorithms that rewrites the order among bodies considering their spatial locality. In this order earth and sun are very liekly to be adjacent in the order than earth and the North star.

2. Use the tree itself. Distance between two bodies is bounded by the diagnol of Lowest common ancestor.  In fact earth and sun are very likely to be adjacent, if we look the bodies we see in the tree dfs Order.  Luckily we are anyway constructing this tree. So we are gonna rewrite the order of bodies we are gonna use to compute the force

The result : A total speed up of over 9x (includes previous optimizations as well). 

The insight : Reduced cache misses across L1/L2 dramatically improves the performance.


### Body Blocking
The idea: 
Since the idea of body reordering is very successful, we take this to the next level. What if we bring the node used commonly by the sun and earth to CPU and do related computation at once for both. We call this idea Body Blocking.

Force compute now looks like : 

```
computeAcceleration ( Block of bodies, Node node)

 for body in Block :
    check if body needs the Node:
      Compute Acceleration of Body, Node as before
```

Of course, we additionally manage the check using bitmasks. This can cause branching overhead, adding extra code for edge cases

Result : A total speed up of over 11x for a block of size 2 (includes previous optimizations)

Insight : Reduced data Movement between L1 and registers. We are computing the result for both earth and sun very likely by keeping the common node data in the register itself. The overhead is worth it. We performed additional analysis
regarding the node reusage. For a block of size 4, define reuse ratio as 

$$ReuseRatio = \frac{\text{Nodes used by } >1 \text{ body in a block}}{\text{Total nodes visited by the function for that block}}$$

In general reuse ratio is barely 5%, with body reordering this shoots up to over 80% even for randomly generated data. For real galaxy data this can likely go even higher, as they include many such star-planet clusters.

### AVX
The idea :
Having seen body blocking improving, it further gives us an idea. What if we compute a block of 4 bodies at once using AVX ?

Result : A total speed up of 20x (includes previous optimization)

Insight : Hard to read code, but worth it in terms of performance. Amortizes the cost of heavy sqrt and div operations involved in the force calculation. Also have to manage the termination conditions by converting back to scalars.


# Correctness & Stability
The baseline implemntation 



All the optimisations produce the exact same output ( the same file diff !) with same exact total Flop count ( except for postCOM ). Indicating the speed up is purely hardware
optimisation, not any extra approximations involving skipping of flop related computations
# How to Build & Run

# Future Work

Multi threading : All the single threaded optimisation can help in improving the multi threaded as well.

Blocked tree construction : Since the tree construction is very small fraction of the total run time, we ignored optimsiing it further. One can apply body blocking even on the tree insertion and gain further improvements.

