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
