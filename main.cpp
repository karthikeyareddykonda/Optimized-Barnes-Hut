#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "libs/baseline.h"
#include "libs/utils.h"
using namespace std;

int main(int argc, char *argv[])
{
    if (argc != 6)
    {
        cout << "Usage a.out inputFile num_iter dt theta outfile\n";
        return 0;
    }
    ifstream File(argv[1]);
    string num_objects;
    getline(File, num_objects);
    const int N = stoi(num_objects);
    const int num_iter = atoi(argv[2]);
    const double dt = stod(argv[3]); // Argv is in seconds --> converted to double
    const double theta = stod(argv[4]);
    const string out_file = argv[5];
    cout << "Num objects " << N << "\n";

    std::string line;
    int body_index = 0;
    vector<Body> Bodies; // Choices, init with Bodies(N) ?
    while (getline(File, line))
    {
        istringstream line_as_stream{line};
        string val;
        string px, py, pz, vx, vy, vz, mass;
        getline(line_as_stream, px, ',');
        getline(line_as_stream, py, ',');
        getline(line_as_stream, pz, ',');
        getline(line_as_stream, vx, ',');
        getline(line_as_stream, vy, ',');
        getline(line_as_stream, vz, ',');
        getline(line_as_stream, mass, ',');

        Vector3D pos(stod(px), stod(py), stod(pz));
        Vector3D vel(stod(vx), stod(vy), stod(vz));

        Bodies.push_back(Body(stod(mass), pos, vel, body_index)); // use emplace_back() ?

        body_index++;
    }

    File.close();
    auto limits = find_min_max(Bodies);
    double width = limits.second - limits.first;
    Vector3D p0_root{limits.first - 1.5 * width, limits.first - 1.5 * width, limits.first - 1.5 * width};

    Node *root = new Node();
    root->p0 = p0_root;
    root->width = 4 * width;
    vector<Vector3D> Accelerations(2 * N);
    Statistics times;
    for (uint i = 0; i < num_iter; i++)
    {
        Statistics times_per_step = timestep(Bodies, Accelerations, root, dt, theta, (i % 2) * N); // Choice : Use a class of parameters. dt, theta etc
        times.t_insert += times_per_step.t_insert;
        times.t_force += times_per_step.t_force;
        times.t_leapfrog += times_per_step.t_leapfrog;
    }

    times.print();
    write_to_file(Bodies, out_file);
}
