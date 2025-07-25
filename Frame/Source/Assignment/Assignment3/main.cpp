#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

#include "Utils.hpp"

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
 
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float fovy_in_radians = eye_fov * MY_PI / 180.0f;
    float tan_half_fovy = std::tan(fovy_in_radians / 2.0f);

    // 标准透视投影矩阵
    projection(0, 0) = 1.0f / (aspect_ratio * tan_half_fovy);
    projection(1, 1) = 1.0f / tan_half_fovy;
    projection(2, 2) = -(zFar + zNear) / (zFar - zNear);
    projection(2, 3) = -2.0f * zFar * zNear / (zFar - zNear);
    projection(3, 2) = -1.0f;
    projection(3, 3) = 0.0f;

    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};


Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = { 0, 0, 0 };
    if (payload.texture)
    {
       
        return_color = payload.texture->getColor(payload.tex_coords.x(), payload.tex_coords.y());

    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights)
    {
    
        Eigen::Vector3f light_dir = light.position - point;
        Eigen::Vector3f view_dir = eye_pos - point;
        float r = light_dir.dot(light_dir);
        // ambient
        Eigen::Vector3f La = ka.cwiseProduct(amb_light_intensity);
        // diffuse
        Eigen::Vector3f Ld = kd.cwiseProduct(light.intensity / r);
        Ld *= std::max(0.0f, normal.normalized().dot(light_dir.normalized()));
        // specular
        Eigen::Vector3f h = (light_dir + view_dir).normalized();
        Eigen::Vector3f Ls = ks.cwiseProduct(light.intensity / r);
        Ls *= std::pow(std::max(0.0f, normal.normalized().dot(h)), p);

        result_color += (La + Ld + Ls);
    }

    return result_color * 255.f;
}
    

Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    struct light {
        Eigen::Vector3f position;
        Eigen::Vector3f intensity;
    };

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal.normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };
    for (auto& light : lights)
    {
        Eigen::Vector3f l = light.position - point;
        Eigen::Vector3f v = eye_pos - point;
        Eigen::Vector3f h = (v + l).normalized();

        float r = l.norm(); // 光源到物体的距离
        float attenuation = 1.0f / (r * r); // 光强衰减

        // Ambient light
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // Diffuse light
        float diff = std::max(0.0f, normal.dot(l.normalized()));
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity) * diff * attenuation;

        // Specular light
        float spec = std::pow(std::max(0.0f, normal.dot(h)), p);
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) * spec * attenuation;

        result_color += ambient + diffuse + specular;
    }

    return result_color * 255.f;
}
Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    auto n = normal;
    auto x = n.x();
    auto y = n.y();
    auto z = n.z();
    Eigen::Vector3f t(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    Eigen::Vector3f b = n.cwiseProduct(t);
    Eigen::Matrix3f TBN;
    TBN.col(0) = t;
    TBN.col(1) = b;
    TBN.col(2) = n;
    float u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float width = payload.texture->width;
    float height = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColorBilinear(std::min(u + 1.f / width, 1.f), v).norm() - \
        payload.texture->getColorBilinear(u, v).norm()
        );

    float dV = kh * kn * (payload.texture->getColorBilinear(u, std::min(v + 1.f / height, 1.f)).norm() - \
        payload.texture->getColorBilinear(u, v).norm()
        );
    Vector3f ln{ -dU,-dV,1 };
   

    //Changge the points real space
    point += kn * n * payload.texture->getColorBilinear(payload.tex_coords.x(), payload.tex_coords.y()).norm();

    // 求世界坐标下的法线，要把切线空间里的法线
    // Normal n = normalize(TBN * ln) 
    normal = (TBN * ln).normalized();


    Eigen::Vector3f result_color = { 0, 0, 0 };

    for (auto& light : lights)
    {
        Eigen::Vector3f l = light.position - point;
        Eigen::Vector3f v = eye_pos - point;
        Eigen::Vector3f h = (v + l).normalized();

        float r = l.norm(); // 光源到物体的距离
        float attenuation = 1.0f / (r * r); // 光强衰减

        // Ambient light
        Eigen::Vector3f ambient = ka.cwiseProduct(amb_light_intensity);

        // Diffuse light
        float diff = std::max(0.0f, normal.dot(l.normalized()));
        Eigen::Vector3f diffuse = kd.cwiseProduct(light.intensity) * diff * attenuation;

        // Specular light
        float spec = std::pow(std::max(0.0f, normal.dot(h)), p);
        Eigen::Vector3f specular = ks.cwiseProduct(light.intensity) * spec * attenuation;

        result_color += ambient + diffuse + specular;
    }

    
    return result_color * 255.f;
}

Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
    auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

    std::vector<light> lights = { l1, l2 };
    Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
    Eigen::Vector3f eye_pos{ 0, 0, 10 };

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    auto n = normal;
    auto x = n.x();
    auto y = n.y();
    auto z = n.z();
    Eigen::Vector3f t(x * y / sqrt(x * x + z * z), sqrt(x * x + z * z), z * y / sqrt(x * x + z * z));
    Eigen::Vector3f b = n.cwiseProduct(t);
    Eigen::Matrix3f TBN;
    TBN.col(0) = t;
    TBN.col(1) = b;
    TBN.col(2) = n;
    float u = payload.tex_coords.x(), v = payload.tex_coords.y();
    float width = payload.texture->width;
    float height = payload.texture->height;
    float dU = kh * kn * (payload.texture->getColor(std::min(u + 1.f / width, 1.f), v).norm() - \
        payload.texture->getColor(u, v).norm()
        );
    float dV = kh * kn * (payload.texture->getColor(u, std::min(v + 1.f / height, 1.f)).norm() - \
        payload.texture->getColor(u, v).norm()
        );

    //caluclate the normal through this function.
    Vector3f ln{ -dU,-dV,1 };

    //Change it back to wordspace,do not forget to normalize it.
    // Normal n = normalize(TBN * ln)   
    normal = (TBN * ln).normalized();

    Eigen::Vector3f result_color = { 0, 0, 0 };
    result_color = normal;
    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;

    // Load .obj File
    bool loadout = Loader.LoadFile(Utils::PathFromAsset("model/spot/spot_triangulated_good.obj"));
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    r.set_texture(Texture(Utils::PathFromAsset("model/spot/spot_texture.PNG")));
    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = texture_fragment_shader;
    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            r.set_texture(Texture(Utils::PathFromAsset("model/spot/spot_texture.png")));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = { 0, 0, 10 }; 
    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }
    }
    return 0;
}
