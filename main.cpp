#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "libs/baseline.h"
#include "libs/utils.h"
using namespace std;

#ifndef SIM_CLASS
#define SIM_CLASS BaseSim
#endif

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

    BaseSim *sim = new SIM_CLASS(dt, theta);
    Statistics times = sim->run(Bodies, num_iter);

    times.print();
    write_to_file(Bodies, out_file);
}
