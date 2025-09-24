#include <Eigen/Dense>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace eth_trajectory_generation
{

class Box3D {

    public:

        std::array<Eigen::Vector3d, 8> vertices;
        std::string frame_id;
        visualization_msgs::Marker   vis_marker;
        double height;

    Box3D() = default;

    Box3D(const std::array<Eigen::Vector3d, 4>& base, double h, std::string box_frame_id) {
        
        this->height = h;

        // find min z of the base and create bottom face at that z
        double min_z = base[0].z();
        for (int i = 1; i < 4; ++i) {
            if (base[i].z() < min_z) {
                min_z = base[i].z();
            }
        }

        // Vertices 0 - 3 are the base, 4 - 7 are the top
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d v = base[i];
            v.z() = min_z;
            vertices[i] = v;
        }
        
        // Create top vertices by adding height to z-coordinate
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d top = base[i];
            top.z() += height;
            vertices[i + 4] = top;
        }
        
        frame_id = box_frame_id;
    }

    bool contains(double x, double y, double z) const {

        double min_z = vertices[0].z();
        double max_z = vertices[4].z();
        if (z < min_z || z > max_z) return false;

        Eigen::Vector2d p(x, y);
        
        Eigen::Vector2d centroid(0.0, 0.0);

        int n = 4; // number of vertices in the base       
        
        for (int i = 0; i < n; ++i)
            centroid += Eigen::Vector2d(vertices[i].x(), vertices[i].y());
        centroid /= double(n);

        const double eps = 1e-9;
        
        for (int i = 0; i < n; ++i) {
            Eigen::Vector2d a(vertices[i].x(), vertices[i].y());
            Eigen::Vector2d b(vertices[(i+1)%n].x(), vertices[(i+1)%n].y());
            Eigen::Vector2d e = b - a;

            Eigen::Vector2d nvec(e.y(), -e.x());

            // Check if the normal vector points inward half-plane
            double s_centroid = nvec.dot(centroid - a);
            if (s_centroid > 0.0) {
                nvec = -nvec;
            }
            // Test point
            double d = nvec.dot(p - a);
            if (d > eps)
                return false;
        }

        return true;
    }
};
}  // namespace eth_trajectory_generation