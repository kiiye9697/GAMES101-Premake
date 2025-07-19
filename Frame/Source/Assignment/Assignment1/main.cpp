#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
//这个为相应的视口变换
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}
//这个步骤为MVP变换中的“M”方法，用于转换
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // 创建一个单位矩阵
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // 将角度转换为弧度
    float angle_in_radians = rotation_angle * MY_PI / 180.0f;

    // 直接构造绕 Z 轴的旋转矩阵
    model(0, 0) = std::cos(angle_in_radians);
    model(0, 1) = -std::sin(angle_in_radians);
    model(1, 0) = std::sin(angle_in_radians);
    model(1, 1) = std::cos(angle_in_radians);

    return model;
}
//需要在这里根据接口方法实现相应的透视投影矩阵
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
     // 将视场角从度转换为弧度,这一步一定要从头贯彻到尾，在这一对应的CPP库当中，要注意所有的角度都是采用弧度制的！
    float fovy_in_radians = eye_fov * MY_PI / 180.0f;

    // 计算半角的正切值
    float tan_half_fovy = std::tan(fovy_in_radians / 2.0f);

    // 设置透视投影矩阵的元素
    projection(0, 0) = 1.0f / (aspect_ratio * tan_half_fovy); // x 缩放
    projection(1, 1) = 1.0f / tan_half_fovy;            // y 缩放
    projection(2, 2) = (zFar + zNear) / (zFar - zNear);     // z 缩放
    projection(2, 3) = (2.0f * zFar * zNear) / (zFar - zNear); // z 偏移
    projection(3, 2) = -1.0f;                           // z 坐标变换
    projection(3, 3) = 0.0f;                            // w 坐标变换

    return projection;
}

//PLUS TASK:USE FIXED Position to get a rotation
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f rotation= Eigen::Matrix4f::Identity();
    // 将角度转换为弧度
    float angle_in_radians = angle * MY_PI / 180.0f;

    // 将轴向量归一化
    axis.normalize();

    // 提取轴向量的分量
    float u_x = axis.x();
    float u_y = axis.y();
    float u_z = axis.z();

    // 计算 sin 和 cos
    //这个地方用标准库的三角函数解决相应问题。
    float sin_theta = std::sin(angle_in_radians);
    float cos_theta = std::cos(angle_in_radians);
    float one_minus_cos_theta = 1.0f - cos_theta;
    //对对应旋转矩阵根据罗德里格斯旋转公式构建旋转矩阵
    rotation(0, 0) = cos_theta + u_x * u_x * one_minus_cos_theta;
    rotation(0, 1) = u_x * u_y * one_minus_cos_theta - u_z * sin_theta;
    rotation(0, 2) = u_x * u_z * one_minus_cos_theta + u_y * sin_theta;

    rotation(1, 0) = u_y * u_x * one_minus_cos_theta + u_z * sin_theta;
    rotation(1, 1) = cos_theta + u_y * u_y * one_minus_cos_theta;
    rotation(1, 2) = u_y * u_z * one_minus_cos_theta - u_x * sin_theta;

    rotation(2, 0) = u_z * u_x * one_minus_cos_theta - u_y * sin_theta;
    rotation(2, 1) = u_z * u_y * one_minus_cos_theta + u_x * sin_theta;
    rotation(2, 2) = cos_theta + u_z * u_z * one_minus_cos_theta;

    return rotation;
 
}
int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    //Update：设计一个转轴用于测试罗德里格斯公式是否使用成功
    //验证罗德里格斯公式是否正确的最好方法就是将旋转轴进行特殊的xyz旋转方法。
    Eigen::Vector3f axis= { 2, 1, 3 };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        //在这里用罗德里格斯旋转轴替换原有的z转轴方法：
        r.set_model(get_rotation(axis,angle));
        //这里保留原有的model方法
        //r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
