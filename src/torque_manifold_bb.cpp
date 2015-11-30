#include <Eigen/Dense>
#include <stdio.h>
#include <string>
#include <fstream>

int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr, "Usage: torque_manifold_bb <filename>\n");
        return 1;
    }

    std::string fname(argv[1]);

    std::ifstream ifs(fname);

    if (!ifs.is_open()) {
        fprintf(stderr, "Failed to open '%s' for reading\n", fname.c_str());
        return 1;
    }

    std::vector<Eigen::Vector3d> points;
    double x, y, z;
    while (ifs >> x >> y >> z) {
        points.push_back(Eigen::Vector3d(x, y, z));
    }

    if (points.empty()) {
        fprintf(stderr, "No points to compute bounding box\n");
        return 1;
    }

    Eigen::Vector3d min_pt = points.front();
    Eigen::Vector3d max_pt = points.front();

    for (const Eigen::Vector3d& pt : points) {
        if (pt.x() < min_pt.x()) {
            min_pt.x() = pt.x();
        }
        if (pt.y() < min_pt.y()) {
            min_pt.y() = pt.y();
        }
        if (pt.z() < min_pt.z()) {
            min_pt.z() = pt.z();
        }

        if (pt.x() > max_pt.x()) {
            max_pt.x() = pt.x();
        }
        if (pt.y() > max_pt.y()) {
            max_pt.y() = pt.y();
        }
        if (pt.z() > max_pt.z()) {
            max_pt.z() = pt.z();
        }
    }

    printf("Min Point: (%lf %lf %lf)\n", min_pt.x(), min_pt.y(), min_pt.z());
    printf("Max Point: (%lf %lf %lf)\n", max_pt.x(), max_pt.y(), max_pt.z());

    return 0;
}
