#include <contact_sensing.hpp>

void ContactLocation::LoadSTL(std::string filename){
    // Open model STL file
    char buffer[BUFFER_SIZE];
    FILE *file = fopen(filename.c_str(),"r");
    if(!file){
        std::cout << "ERROR! Can not open STL file.";
        std::exit(0);
    }

    // Read model STL file
    std::vector <std::string> normalIn;
    std::vector <std::string> vertexIn;
    // Output model name
    char modelname[BUFFER_SIZE];
    fgets(buffer, BUFFER_SIZE, file);
    if(buffer[0]=='s'){
        sscanf(buffer, "%*s %s", modelname);
        // std::cout << "Reading : " << modelname << ".STL" << std::endl;
    }
    while (fgets(buffer, BUFFER_SIZE, file) != NULL)
    {
        int ptr = 0;
        while (buffer[ptr] == ' ') ptr++;
        if (buffer[ptr] == 'f') normalIn.push_back(std::string(buffer));
        if (buffer[ptr] == 'v') vertexIn.push_back(std::string(buffer));
    }
    fclose(file);

    // Save face normal and vertex
    faceN = normalIn.size();
    Eigen::Vector3d iter_normal;
    Eigen::Matrix3d iter_vertex;
    Eigen::Vector4d iter_facePara;
    for (int i=0; i<faceN; i++){
        sscanf(normalIn[i].c_str(),"%*s %*s %lf %lf %lf", &iter_normal[0], &iter_normal[1], &iter_normal[2]);
        normal.push_back(iter_normal);

        for (int j=0; j<3; j++){
            sscanf(vertexIn[i*3+j].c_str(),"%*s %lf %lf %lf", &iter_vertex(j,0), &iter_vertex(j,1), &iter_vertex(j,2));
        }
        vertex.push_back(iter_vertex);

        iter_facePara[0] = iter_normal[0]; // a
        iter_facePara[1] = iter_normal[1]; // b
        iter_facePara[2] = iter_normal[2]; // c
        iter_facePara[3] = -(iter_normal[0]*iter_vertex(0,0)+iter_normal[1]*iter_vertex(0,1)+iter_normal[2]*iter_vertex(0,2));
        facePara.push_back(iter_facePara);

    }

}

bool ContactLocation::calContactPoint(Eigen::Matrix<double,6,1> &force, Eigen::Vector3d &point){
    Eigen::Matrix4d A;
    Eigen::Vector4d B, X;
    Eigen::Vector3d n0, v1, v2, v3;
    Eigen::Vector3d p0, p1, p2, p3, f;

    Eigen::Vector3d iter_normal;
    Eigen::Matrix3d iter_vertex;
    Eigen::Vector4d iter_facePara;
    
    // Compute CoLD equation
    for (int i=0; i<faceN; i++){
        // Load surface
        n0 = normal[i];
        v1 << vertex[i](0,0),vertex[i](0,1),vertex[i](0,2);
        v2 << vertex[i](1,0),vertex[i](1,1),vertex[i](1,2);
        v3 << vertex[i](2,0),vertex[i](2,1),vertex[i](2,2);

        // Matrix solution
        A << 0.0000, force[2], -force[1], facePara[i](0),
            -force[2], 0.00000, force[0], facePara[i](1),
            force[1], -force[0], 0.00000, facePara[i](2),
            facePara[i](0), facePara[i](1), facePara[i](2), 0;
        // std::cout << A << std::endl;
        B << force[3], force[4], force[5], -facePara[i](3);
        X = A.inverse()*B;

        // Calculate surface limitations
        p0 << X(0), X(1), X(2);
        p1 = v1-p0;p1.normalize();
        p2 = v2-p0;p2.normalize();
        p3 = v3-p0;p3.normalize();
        if (fabs(acos(p1.dot(p2))+acos(p1.dot(p3))+acos(p2.dot(p3))-2*M_PI)<0.1){
            f << force[0], force[1], force[2];
            if (f.dot(n0)<0){
                point = p0;
                return 1;
            }
        }
    }

    // Add another plane for more stable control
    n0 << 0, 0, 1;

    // Matrix solution
    A << 0.0000, force[2], -force[1], 0,
        -force[2], 0.00000, force[0], 0,
        force[1], -force[0], 0.00000, 1,
        0, 0, 1, 0;
    // std::cout << A << std::endl;
    B << force[3], force[4], force[5], 0.188;
    X = A.inverse()*B;

    // Calculate surface limitations
    p0 << X(0), X(1), X(2);
    if(fabs(p0[0])<0.01){
        if(fabs(p0[1]<0.02)){
            point = p0;
            return 1;
        }
    }

    return 0;
}