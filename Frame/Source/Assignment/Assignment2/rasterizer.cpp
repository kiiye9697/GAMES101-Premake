// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Eigen::Vector3f v0 = _v[0];
    Eigen::Vector3f v1 = _v[1];
    Eigen::Vector3f v2 = _v[2];

    // 计算边向量
    Eigen::Vector3f ab = v1 - v0;
    Eigen::Vector3f bc = v2 - v1;
    Eigen::Vector3f ca = v0 - v2;

    // 计算点相对于顶点的位置向量
    Eigen::Vector3f ap = Eigen::Vector3f(x - v0.x(), y - v0.y(), 0.0f);
    Eigen::Vector3f bp = Eigen::Vector3f(x - v1.x(), y - v1.y(), 0.0f);
    Eigen::Vector3f cp = Eigen::Vector3f(x - v2.x(), y - v2.y(), 0.0f);

    // 计算叉乘
    Eigen::Vector3f cross1 = ab.cross(ap);
    Eigen::Vector3f cross2 = bc.cross(bp);
    Eigen::Vector3f cross3 = ca.cross(cp);

    // 检查叉乘结果的z分量的符号是否一致
    return (cross1.z() >= 0 && cross2.z() >= 0 && cross3.z() >= 0) ||
        (cross1.z() <= 0 && cross2.z() <= 0 && cross3.z() <= 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // Find out the bounding box of current triangle.
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    //Step 1:calculate the start_point and the end_point.
    //其中v是一个三维顶点，传入计算的时候被设定为一个齐次坐标。
    float x_min = std::min({ v[0].x(), v[1].x(), v[2].x() });
    float y_min = std::min({ v[0].y(), v[1].y(), v[2].y() });
    float x_max = std::max({ v[0].x(), v[1].x(), v[2].x() });
    float y_max = std::max({ v[0].y(), v[1].y(), v[2].y() });

    int x_start = static_cast<int>(std::floor(x_min));
    int y_start = static_cast<int>(std::floor(y_min));
    int x_end = static_cast<int>(std::ceil(x_max));
    int y_end = static_cast<int>(std::ceil(y_max));
    //Step2：check each pixal in the bounding box to ensure if is inside.
    for (int x = x_start; x < x_end; ++x)
    {
        for (int y = y_start; y < y_end; ++y) {
            if (x < 0 || x >= width || y < 0 || y >= height) {
                continue;
            }

            Eigen::Vector3f color_sum(0.0f, 0.0f, 0.0f);
            float depth_sum = 0.0f;
            int sample_count = 0;

            // 对每个像素进行 2x2 采样，每个位置都保留原点位置左上，左下，右上，右下像素点再取平均值。
            for (int sy = 0; sy < 2; ++sy) {
                for (int sx = 0; sx < 2; ++sx) {
                    float sub_x = x + (sx + 0.5f) / 2.0f;
                    float sub_y = y + (sy + 0.5f) / 2.0f;

                    if (!insideTriangle(static_cast<int>(sub_x), static_cast<int>(sub_y), t.v)) 
                    {
                        continue;
                        //here，when oversampling，some place may out of it though the origin point is inside,which better the performance.
                    }

                    auto [alpha, beta, gamma] = computeBarycentric2D(sub_x, sub_y, t.v);
                    float w_reciprocal = 1.0f / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated =
                        alpha * v[0].z() / v[0].w() +
                        beta * v[1].z() / v[1].w() +
                        gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;

                    color_sum += t.getColor();
                    depth_sum += z_interpolated;
                    sample_count++;
                }
            }

            if (sample_count > 0) {
                color_sum /= sample_count;
                depth_sum /= sample_count;

                int ind = get_index(x, y);
                if (depth_sum<depth_buf[ind])
                {
                    depth_buf[ind] = depth_sum;
                    Eigen::Vector3f point(x, y, depth_sum);
                    set_pixel(point, color_sum);
                }
            }
        }
    }
}


void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on
