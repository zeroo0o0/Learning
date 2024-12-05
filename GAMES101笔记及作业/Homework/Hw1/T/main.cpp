#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;
constexpr double DEG2RAD = MY_PI / 180.0;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();  // 4维单位矩阵

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    const float angle = rotation_angle * (float)DEG2RAD;
    model << cos(angle), -sin(angle), 0, 0, //旋转矩阵 
             sin(angle), cos(angle),  0, 0,
             0,          0,           1, 0,
             0,          0,           0, 1;

    return model;
}

// 提高项
// 应用罗德里格斯旋转公式
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    // 旋转的角度
    float rotation_angle = angle * DEG2RAD;
    // 最终的罗德里格斯矩阵
    Eigen::Matrix4f rotation_matrix;

    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();    // 3维单位矩阵
    Eigen::Matrix3f R;  // 三维的罗德里格斯矩阵 （即绕任意轴旋转的旋转矩阵） 
    Eigen::Matrix3f N;  // axis旋转轴化作矩阵 （叉乘） 

    N << 0,         -axis[2], axis[1],  //axis 向量(nx, ny, nz) 
         axis[2],   0,        -axis[0],
         -axis[1],  axis[0],  0;

    R = cos(rotation_angle) * I + (1 - cos(rotation_angle)) * axis * axis.transpose() + sin(rotation_angle) * N;

    rotation_matrix << R(0, 0), R(0, 1), R(0, 2), 0,   //R(0,0) 矩阵的第0行第0列的数 
                       R(1, 0), R(1, 1), R(1, 2), 0,
                       R(2, 0), R(2, 1), R(2, 2), 0,
                       0,       0,       0,       1;

    return rotation_matrix;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, //eye_fov 视角  aspect_ratio 宽/高  
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // zNear 和 zFar 都是负值 （但top是正数，所以下面公式加了-） 
    const float top = tan(DEG2RAD * (eye_fov / 2.0f)) * -(zNear);  //eye_fov:视角 
    const float right = aspect_ratio * top;  //aspect_ratio :宽 / 高 

    projection << zNear/right, 0,         0,                         0,
                  0,           zNear/top, 0,                         0,  //r-l：2r    t-b=2t  因为r，t正数 
                  0,           0,         (zNear+zFar)/(zNear-zFar), (2*zNear*zFar)/(zFar-zNear),
                  0,           0,         1,                         0;

    return projection;
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

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        Eigen::Vector3f axis(0, 0, 1);
        r.set_model(get_rotation(axis, angle));
        /* r.set_model(get_model_matrix(angle)); */
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

        Eigen::Vector3f axis(0, 0, 1);
        r.set_model(get_rotation(axis, angle));
        /* r.set_model(get_model_matrix(angle)); */
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
