#include <Eigen/Dense>
#include <iostream>

namespace eth_trajectory_generation
{

class Box3D {

    public:
    std::array<Eigen::Vector3d, 8> vertices; // 8 vrcholů boxu

    // Konstruktor: podstava (4 body, všechny mají stejnou Z) + výška
    Box3D(const std::array<Eigen::Vector3d, 4>& base, double height) {
        // Podstava
        for (int i = 0; i < 4; ++i) {
            vertices[i] = base[i];
        }
        // Horní podstava (posun Z o height)
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector3d top = base[i];
            top.z() += height;
            vertices[i + 4] = top;
        }
    }

    // Test, zda bod leží uvnitř boxu (zjednodušeně: projekce do XY a test Z)
    bool contains(double x, double y, double z) const {
        // Zjisti min/max Z
        double min_z = vertices[0].z();
        double max_z = vertices[4].z();

        if (z < min_z || z > max_z)
            return false;

        // Projekce bodu do XY a test, zda leží uvnitř podstavy (polygon test)
        Eigen::Vector2d p(x, y);
        int crossings = 0;
        for (int i = 0; i < 4; ++i) {
            Eigen::Vector2d a(vertices[i].x(), vertices[i].y());
            Eigen::Vector2d b(vertices[(i+1)%4].x(), vertices[(i+1)%4].y());
            if (((a.y() > p.y()) != (b.y() > p.y())) &&
                (p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()))
                crossings++;
        }
        return (crossings % 2 == 1);
    }
};
}  // namespace eth_trajectory_generation