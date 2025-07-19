#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>

int main(){

    //// Basic Example of cpp
    //std::cout << "Example of cpp \n";
    //float a = 1.0, b = 2.0;
    //std::cout << a << std::endl;
    //std::cout << a/b << std::endl;
    //std::cout << std::sqrt(b) << std::endl;
    //std::cout << std::acos(-1) << std::endl;
    //std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    //// Example of vector
    //std::cout << "Example of vector \n";
    //// vector definition
    //Eigen::Vector3f v(1.0f,2.0f,3.0f);
    //Eigen::Vector3f w(1.0f,0.0f,0.0f);
    //// vector output
    //std::cout << "Example of output \n";
    //std::cout << v << std::endl;
    //// vector add
    //std::cout << "Example of add \n";
    //std::cout << v + w << std::endl;
    //// vector scalar multiply
    //std::cout << "Example of scalar multiply \n";
    //std::cout << v * 3.0f << std::endl;
    //std::cout << 2.0f * v << std::endl;

    //// Example of matrix
    //std::cout << "Example of matrix \n";
    //// matrix definition
    //Eigen::Matrix3f i,j;
    //i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    //j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    //// matrix output
    //std::cout << "Example of output \n";
    //std::cout << i+j << std::endl;
    //// matrix add i + j
    //// matrix scalar multiply i * 2.0
    //// matrix multiply i * j
    //// matrix multiply vector i * v
    
    // 定义点P的齐次坐标(点的第三维齐次坐标为1，在坐标变换时也会发生对应改变)

 
        // 定义点P的齐次坐标
        Eigen::Vector3d P_h(2, 1, 1);

        // 定义旋转矩阵（逆时针旋转45度）
        double angle = 45.0 * EIGEN_PI / 180.0; // 将角度转换为弧度
        Eigen::Matrix3d R;
        R << cos(angle), -sin(angle), 0,
            sin(angle), cos(angle), 0,
            0, 0, 1;

        // 定义平移矩阵（平移(1, 2)）
        Eigen::Matrix3d T;
        T << 1, 0, 1,
            0, 1, 2,
            0, 0, 1;

        // 先旋转，再平移
        Eigen::Vector3d P_h_prime = R * P_h; // 旋转
        Eigen::Vector3d P_h_double_prime = T * P_h_prime; // 平移

        // 输出变换后的点
        std::cout << "变换后的点P''的齐次坐标为: " << std::endl;
        std::cout << P_h_double_prime << std::endl;

        // 转换为笛卡尔坐标
        Eigen::Vector2d P_double_prime_cartesian(P_h_double_prime(0), P_h_double_prime(1));
        std::cout << "变换后的点P''的笛卡尔坐标为: ("
            << P_double_prime_cartesian(0) << ", "
            << P_double_prime_cartesian(1) << ")" << std::endl;

        return 0;
   
}