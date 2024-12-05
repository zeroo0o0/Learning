#include <chrono>
#include<cmath> //头文件
#include <iostream>
#include<opencv2/core/matx.hpp>
#include<opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
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

//说白了就是固定是三阶了（四个控制点）
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

		// 曲线颜色设置为红色，OpenCV为 BGR
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// 贝塞尔曲线的基本公式 //封装成函数方便调用
Point2f lerp_v2f(const Point2f& a,const Point2f& b,float t)
{
	return (1-t)*a+t*b;  //虽然t是靠近a点那段的比例
}

//   递归贝塞尔曲线 //相当于一个一个地根据t找点并且画出
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //结束条件
    if(control_points.size()==1) return control_points[0];
    // 新建一个数组，储存新的控制点，数量每次会减少 1，减少到 1时即为出口
	vector<Point2f>lerp_points;
    // 遍历原控制点数组，将通过 lerp_v2f（最基本的公式）新形成的控制点依次填入新的数组
	for(size_t i=1;i<control_points.size();i++)
	{
		lerp_points.push_back(lerp_v2f(control_points[i-1],control_points[i],t));
	}
	//继续递归
	return recursive_bezier(lerp_points,t);
}

//遍历参数 t
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.   递归贝塞尔
	for(double t=0.0;t<=1.0;t+=0.001)
	{
        // 对于每一个 t，求它对应的贝塞尔曲线的点，这里是通过最基本的公式 (1 - t) * a + t * b，递归求得，
        // 实际上也可以直接用 n阶展开式，直接求得，就像 naive_bezier函数那样		
		auto point =recursive_bezier(control_points,t);
		window.at<Vec3b>(point.y, point.x)[2]=255; //画出
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
            //bezier(control_points, window);

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
