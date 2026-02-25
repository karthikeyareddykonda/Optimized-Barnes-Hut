#include "baseline.h"
#include "utils.h"
#include "immintrin.h"
#include "math.h"

void AVXSim::compute_acceleration_block(const Body *bodies, const Node *const root, Vector3D *accelerations)
{
    std::vector<std::pair<const Node *, int>> rec_stack;
    rec_stack.push_back(std::make_pair(root, (1 << BLOCK_SIZE) - 1));

    // Warning algorithm guaranteed only for num_bodies >= 2 and theta <= 1
    __m256d b_x = _mm256_set_pd(bodies[3].pos.x, bodies[2].pos.x, bodies[1].pos.x, bodies[0].pos.x);
    __m256d b_y = _mm256_set_pd(bodies[3].pos.y, bodies[2].pos.y, bodies[1].pos.y, bodies[0].pos.y);
    __m256d b_z = _mm256_set_pd(bodies[3].pos.z, bodies[2].pos.z, bodies[1].pos.z, bodies[0].pos.z);

    __m256d a_x = _mm256_set1_pd(0);
    __m256d a_y = _mm256_set1_pd(0);
    __m256d a_z = _mm256_set1_pd(0);

    while (!rec_stack.empty())
    {
        // total_loops++;

        const Node *const parent = rec_stack.back().first;
        int mask = rec_stack.back().second;
        rec_stack.pop_back();
        // if ((mask & (mask - 1)) != 0)
        //     adv++;
        //  std::cout << "node depth : " << parent->width << "\n";
        for (unsigned int i = 0; i < 8; i++)
        {
            int sub_mask = mask;
            const Node *node = parent->get_child(i); // Incorrect semantics. parent itself maybe a leaf!
            //  std::cout << "child : " << node << "\n";

            if (node->is_leaf())
            {
                if (node->body == nullptr)
                    continue; // skip the node

                /*
                __m256i v_bodies = _mm256_set_epi64x(
                    (intptr_t)(bodies + 3),
                    (intptr_t)(bodies + 2),
                    (intptr_t)(bodies + 1),
                    (intptr_t)(bodies + 0));

                __m256i v_node_body = _mm256_set1_epi64x((intptr_t)node->body);

                __m256i v_cmp = _mm256_cmpeq_epi64(v_bodies, v_node_body);

                int match_mask = _mm256_movemask_pd(_mm256_castsi256_pd(v_cmp));

                sub_mask &= ~match_mask;

                */
                for (uint bind = 0; bind < BLOCK_SIZE; bind++)
                {
                    const Body *b = bodies + bind;
                    int same_body = (b == node->body);
                    if (((sub_mask >> bind) & 1) && b == node->body)
                        sub_mask ^= (1 << bind);
                }

                if (sub_mask == 0)
                    continue; // Bail out from flops

                __m256d com_x = _mm256_set1_pd(node->COM.x);
                __m256d com_y = _mm256_set1_pd(node->COM.y);
                __m256d com_z = _mm256_set1_pd(node->COM.z);

                __m256d xdiff = _mm256_sub_pd(b_x, com_x);
                __m256d ydiff = _mm256_sub_pd(b_y, com_y);
                __m256d zdiff = _mm256_sub_pd(b_z, com_z);

                __m256d xdiff_sq = _mm256_mul_pd(xdiff, xdiff);
                __m256d ydiff_sq = _mm256_mul_pd(ydiff, ydiff);
                __m256d zdiff_sq = _mm256_mul_pd(zdiff, zdiff);
                __m256d xdiff_ydiff_sum = _mm256_add_pd(xdiff_sq, ydiff_sq);
                __m256d r_sq = _mm256_fmadd_pd(zdiff, zdiff, xdiff_ydiff_sum);
                __m256d r = _mm256_sqrt_pd(r_sq);

                __m256d fac = _mm256_set1_pd(-G * node->mass);
                // __m256d mask = get_1_0_multiplier(sub_mask);
                __m256d mask = mask_from_4bit_opt(sub_mask); // _mm256_mul_pd(effective_mass, mask_mul);
                __m256d r_3 = _mm256_mul_pd(r_sq, r);
                fac = _mm256_div_pd(fac, r_3); // Some values are NAN !. The mask handles it
                fac = _mm256_and_pd(fac, mask);
                // division by zero handling ! Do the div and then mask

                //
                a_x = _mm256_fmadd_pd(xdiff, fac, a_x);
                a_y = _mm256_fmadd_pd(ydiff, fac, a_y);
                a_z = _mm256_fmadd_pd(zdiff, fac, a_z);

                // No spped up difference between above and below code
                /*
                for (uint bind = 0; bind < BLOCK_SIZE; bind++)
                {
                    if (((sub_mask >> bind) & 1))
                    {
                        const Body *b = bodies + bind;
                        Vector3D b_p = b->pos;
                        Vector3D COM = node->COM;
                        Vector3D r{b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
                        double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
                        double fac = -G * node->body->mass / (r_norm * r_norm * r_norm);
                        Vector3D contrib{fac * r.x, fac * r.y, fac * r.z};
                        *(accelerations + bind) += contrib;
                    }
                }
                    */
                continue;
            }

            __m256d com_x = _mm256_set1_pd(node->COM.x);
            __m256d com_y = _mm256_set1_pd(node->COM.y);
            __m256d com_z = _mm256_set1_pd(node->COM.z);

            __m256d xdiff = _mm256_sub_pd(b_x, com_x);
            __m256d ydiff = _mm256_sub_pd(b_y, com_y);
            __m256d zdiff = _mm256_sub_pd(b_z, com_z);

            __m256d xdiff_sq = _mm256_mul_pd(xdiff, xdiff);
            __m256d ydiff_sq = _mm256_mul_pd(ydiff, ydiff);
            __m256d zdiff_sq = _mm256_mul_pd(zdiff, zdiff);
            __m256d xdiff_ydiff_sum = _mm256_add_pd(xdiff_sq, ydiff_sq);
            __m256d r_sq = _mm256_fmadd_pd(zdiff, zdiff, xdiff_ydiff_sum);
            __m256d r = _mm256_sqrt_pd(r_sq);
            __m256d r_3 = _mm256_mul_pd(r_sq, r);

            __m256d width_sq = _mm256_set1_pd(node->width * node->width);
            __m256d theta_sq = _mm256_set1_pd(theta * theta);
            __m256d r_theta_sq = _mm256_mul_pd(theta_sq, r_sq);
            __m256d criteria_mask = _mm256_cmp_pd(width_sq, r_theta_sq, _CMP_LT_OS);
            __m256d body_mask = mask_from_4bit_opt(sub_mask);
            __m256d compute_mask = _mm256_and_pd(body_mask, criteria_mask);
            body_mask = _mm256_andnot_pd(criteria_mask, body_mask); // next body_mask

            __m256d effective_mass = _mm256_set1_pd(-G * node->mass);
            __m256d mask_mul = get_1_0_multiplier(sub_mask);
            __m256d fac = _mm256_mul_pd(effective_mass, mask_mul); //_mm256_set_pd(effective_mass * ((sub_mask >> 3) & 1), effective_mass * ((sub_mask >> 2) & 1), effective_mass * ((sub_mask >> 1) & 1), effective_mass * ((sub_mask >> 0) & 1));
            fac = _mm256_div_pd(fac, r_3);                         // Div by zero NAN values !. Mask handles it
            fac = _mm256_and_pd(fac, compute_mask);

            a_x = _mm256_fmadd_pd(xdiff, fac, a_x);
            a_y = _mm256_fmadd_pd(ydiff, fac, a_y);
            a_z = _mm256_fmadd_pd(zdiff, fac, a_z);

            // 2x speed up due to AVX here

            /*
            for (uint bind = 0; bind < BLOCK_SIZE; bind++)
            {

                if (((sub_mask >> bind) & 1))
                {
                    const Body *b = bodies + bind;
                    Vector3D b_p = b->pos;
                    Vector3D COM = node->COM;
                    Vector3D r{b_p.x - COM.x, b_p.y - COM.y, b_p.z - COM.z};
                    double r_sq = r.x * r.x + r.y * r.y + r.z * r.z;
                    double r_norm = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
                    // not end node -> check if width/distance < theta
                    if (node->width * node->width < r_sq * theta * theta)
                    { // less than theta -> compute acceleration between node and body
                        // body approximated out
                        sub_mask ^= (1 << bind);
                        double fac = -G * node->mass / (r_sq * r_norm);
                        Vector3D contrib{fac * r.x, fac * r.y, fac * r.z};
                        *(accelerations + bind) += contrib;
                    }
                }
            }
            */

            sub_mask = _mm256_movemask_pd(body_mask);

            // Push the child to stack
            if (sub_mask != 0 && !node->is_leaf())
            {
                rec_stack.push_back(std::make_pair(node, sub_mask));
            }
        }
    }

    // Once per block of bodies is okay. Not in every node
    __m128d lo = _mm256_extractf128_pd(a_x, 0); //_mm256_extractf64x2_pd(a_x, 0);
    __m128d hi = _mm256_extractf128_pd(a_x, 1);

    accelerations[0].x += _mm_cvtsd_f64(lo);
    accelerations[1].x += _mm_cvtsd_f64(_mm_unpackhi_pd(lo, lo));
    accelerations[2].x += _mm_cvtsd_f64(hi);
    accelerations[3].x += _mm_cvtsd_f64(_mm_unpackhi_pd(hi, hi));

    lo = _mm256_extractf128_pd(a_y, 0);
    hi = _mm256_extractf128_pd(a_y, 1);

    accelerations[0].y += _mm_cvtsd_f64(lo);
    accelerations[1].y += _mm_cvtsd_f64(_mm_unpackhi_pd(lo, lo));
    accelerations[2].y += _mm_cvtsd_f64(hi);
    accelerations[3].y += _mm_cvtsd_f64(_mm_unpackhi_pd(hi, hi));

    lo = _mm256_extractf128_pd(a_z, 0);
    hi = _mm256_extractf128_pd(a_z, 1);

    accelerations[0].z += _mm_cvtsd_f64(lo);
    accelerations[1].z += _mm_cvtsd_f64(_mm_unpackhi_pd(lo, lo));
    accelerations[2].z += _mm_cvtsd_f64(hi);
    accelerations[3].z += _mm_cvtsd_f64(_mm_unpackhi_pd(hi, hi));
}
