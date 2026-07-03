#include <vector>
#include <iostream>
#include <Eigen/Dense>

#define BUFFER_SIZE 1024

class ContactLocation
{
public:
    // Model's parameters
    int faceN; // Face number
    std::vector <Eigen::Vector3d> normal;  // Face Normal, n
    std::vector <Eigen::Matrix3d> vertex; // 3 Vertex, v1-v2-v3
    std::vector <Eigen::Vector4d> facePara;  // Face description(a,b,c,d) : ax+by+cz+d=0

public:
    void LoadSTL(std::string filename);
    bool calContactPoint(Eigen::Matrix<double,6,1> &force, Eigen::Vector3d &point);
};