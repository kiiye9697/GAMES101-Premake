//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        //Update：when use our own texture,we should ensure if we create it successfully.
        if (image_data.empty()) {
            std::cerr << "Failed to load texture image: " << name << std::endl;
            // 处理错误，例如抛出异常或使用默认纹理
            throw std::runtime_error("Texture image not found or failed to load.");
        }
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        //when actually use some model else,we need to ensure we load it right.
        u = std::fmod(u, 1.0f);
        if (u < 0) u += 1.0f;
        v = std::fmod(v, 1.0f);
        if (v < 0) v += 1.0f;
        auto u_img = fabs(u * width);
        auto v_img = fabs((1 - v) * height);
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        u = std::fmod(u, 1.0f);
        if (u < 0) u += 1.0f;
        v = std::fmod(v, 1.0f);
        if (v < 0) v += 1.0f;
        auto u_img = fabs(u * width);
        auto v_img = fabs((1 - v) * height);
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        int u0 = static_cast<int>(std::floor(u_img));
        int v0 = static_cast<int>(std::floor(v_img));
        int u1 = u0 + 1;
        int v1 = v0 + 1;

        // 确保坐标在图像范围内
        u0 = std::max(0, std::min(u0, width - 1));
        v0 = std::max(0, std::min(v0, height - 1));
        u1 = std::max(0, std::min(u1, width - 1));
        v1 = std::max(0, std::min(v1, height - 1));

        // 获取四个角落的颜色值
        cv::Vec3b c00 = image_data.at<cv::Vec3b>(v0, u0);
        cv::Vec3b c01 = image_data.at<cv::Vec3b>(v1, u0);
        cv::Vec3b c10 = image_data.at<cv::Vec3b>(v0, u1);
        cv::Vec3b c11 = image_data.at<cv::Vec3b>(v1, u1);

        // 计算双线性插值的权重
        float tu = u_img - u0;
        float tv = v_img - v0;

        // 在 u 方向进行线性插值
        cv::Vec3b c0 = c00 * (1.0f - tu) + c10 * tu;
        cv::Vec3b c1 = c01 * (1.0f - tu) + c11 * tu;

        // 在 v 方向进行线性插值
        cv::Vec3b c = c0 * (1.0f - tv) + c1 * tv;

        // 返回插值后的颜色
        return Eigen::Vector3f(c[0], c[1], c[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
