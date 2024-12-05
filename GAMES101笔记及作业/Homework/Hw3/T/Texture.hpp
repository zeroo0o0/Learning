//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include "opencv2/core/matx.hpp"
#include <algorithm>
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
	//估计是大佬自己加的
    Eigen::Vector3f getColorBilinear(float u, float v) { 
        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        // 找到附近的四个坐标大小
        int u_min = (int)u_img;
        int u_max = std::min(width, (int)u_img + 1);
        int v_min = (int)v_img;
        int v_max = std::min(height, (int)v_img + 1);

        // 获得周围4个点的颜色
        auto point_A = image_data.at<cv::Vec3b>(v_max, u_min);
        auto point_B = image_data.at<cv::Vec3b>(v_max, u_max);
        auto point_C = image_data.at<cv::Vec3b>(v_min, u_min);
        auto point_D = image_data.at<cv::Vec3b>(v_min, u_max);

        // 点与C点，即左下角的点距离
        float x_to_C = u_img - (float)u_min;
        float y_to_C = v_img - (float)v_min;

        // 先进行row的两个插值(A,B) 和 (C,D)
        auto row_AB = (1 - x_to_C) * point_A + x_to_C * point_B;
        auto row_CD = (1 - x_to_C) * point_C + x_to_C * point_D;
        // 再column插值
        auto color = (1 - y_to_C) * row_CD + y_to_C * row_AB;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
