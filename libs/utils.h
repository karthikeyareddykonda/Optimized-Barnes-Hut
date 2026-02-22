#pragma once
#include "baseline.h"
#include <vector>
#include <cfloat>
#include <fstream>
#include <iomanip>
inline std::pair<double, double> find_min_max(const std::vector<Body> &Bodies)
{
    double res_min = DBL_MAX, res_max = DBL_MIN;
    for (const Body &b : Bodies)
    {
        Vector3D v = b.pos;
        res_min = std::min({v.x, v.y, v.z, res_min});
        res_max = std::max({v.x, v.y, v.z, res_max});
    }
    return std::make_pair(res_min, res_max);
}

inline void write_to_file(const std::vector<Body> &Bodies, std::string filename)
{

    std::ofstream file(filename);
    file << Bodies.size() << "\n";

    for (const auto &b : Bodies)
    {
        file << std::fixed << std::setprecision(10) << b.pos.x << "," << b.pos.y << "," << b.pos.z << "," << b.vel.x << "," << b.vel.y << "," << b.vel.z << "," << b.mass << "\n";
    }

    file << std::flush;
}
