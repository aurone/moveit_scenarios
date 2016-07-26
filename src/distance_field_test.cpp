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

#include <stdio.h>
#include <math.h>

#include <moveit/distance_field/propagation_distance_field.h>

int main(int argc, char* argv[])
{
    // NOTE: assumed that res divides size_n evenly

    double size_x = 10.0;
    double size_y = 10.0;
    double size_z = 1.0;
    double res = 1.0;
    double max_dist = 10.0 * sqrt(2.0);
    double ox, oy, oz;
    ox = oy = oz = 0.0;
    distance_field::PropagationDistanceField dfield(size_x, size_y, size_z, res, ox, oy, oz, max_dist);

    const int x_count = dfield.getXNumCells();
    const int y_count = dfield.getYNumCells();
    const int z_count = dfield.getZNumCells();

    EigenSTL::vector_Vector3d points;
    for (int gx = 0; gx < x_count; ++gx) {
        double cx, cy, cz;

        dfield.gridToWorld(gx, 0, 0, cx, cy, cz);
        points.push_back(Eigen::Vector3d(cx, cy, cz));

        dfield.gridToWorld(gx, y_count - 1, 0, cx, cy, cz);
        points.push_back(Eigen::Vector3d(cx, cy, cz));

        dfield.gridToWorld(gx, y_count >> 1, 0, cx, cy, cz);
        points.push_back(Eigen::Vector3d(cx, cy, cz));
    }

    for (int gy = 0; gy < y_count; ++gy) {
        double cx, cy, cz;

        dfield.gridToWorld(0, gy, 0, cx, cy, cz);
        points.push_back(Eigen::Vector3d(cx, cy, cz));

        dfield.gridToWorld(x_count - 1, gy, 0, cx, cy, cz);
        points.push_back(Eigen::Vector3d(cx, cy, cz));
    }

    printf("adding %zu points to dfield\n", points.size());
    dfield.addPointsToField(points);

    for (int y = 9; y >= 0; --y) {
        for (int x = 0; x < 10; ++x) {
            printf("%2.3f ", dfield.getDistance(x, y, 0));
        }
        printf("\n");
    }
    fflush(stdout);

    return 0;
}
