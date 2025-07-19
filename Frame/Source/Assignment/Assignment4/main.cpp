#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    //Skip place
    if (control_points.size() == 1) {
        return control_points[0];
    }
    std::vector<cv::Point2f> new_control_points;
    //still keep upload
    for (size_t i = 0; i < control_points.size() - 1; ++i) {
        //count each point from this logic
        float x = (1 - t) * control_points[i].x + t * control_points[i + 1].x;
        float y = (1 - t) * control_points[i].y + t * control_points[i + 1].y;
        new_control_points.push_back(cv::Point2f(x, y));
    }

    // roll method
    return recursive_bezier(new_control_points, t);
}

void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window) {
    // 设置步长
    for (float t = 0.0f; t <= 1.0f; t += 0.001f) {
        // 使用de Casteljau算法计算曲线上的点
        cv::Point2f point = recursive_bezier(control_points, t);

        // 计算当前点的整数坐标
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);

        // 遍历当前点周围的 3x3 像素
        for (int i = -1; i <= 1; ++i) {
            for (int j = -1; j <= 1; ++j) {
                int neighbor_x = x + j;
                int neighbor_y = y + i;

                // 检查是否在图像范围内
                if (neighbor_x >= 0 && neighbor_x < window.cols && neighbor_y >= 0 && neighbor_y < window.rows) {
                    // 计算与中心点的距离
                    float distance = std::sqrt(std::pow(point.x - (neighbor_x + 0.5f), 2) + std::pow(point.y - (neighbor_y + 0.5f), 2));

                    // 根据距离确定插值权重，使用线性插值，距离越近权重越高
                    float weight = 1.0f - std::min(distance / 1.4142f, 1.0f); 
                    window.at<cv::Vec3b>(neighbor_y, neighbor_x)[1] += 255 * weight;
                }
            }
        }
    }
}

int main() 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
