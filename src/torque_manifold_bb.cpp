////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Andrew Dornbush

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
