#include <chrono>
#include<cmath> //ͷ�ļ�
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

//˵���˾��ǹ̶��������ˣ��ĸ����Ƶ㣩
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

		// ������ɫ����Ϊ��ɫ��OpenCVΪ BGR
        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

// ���������ߵĻ�����ʽ //��װ�ɺ����������
Point2f lerp_v2f(const Point2f& a,const Point2f& b,float t)
{
	return (1-t)*a+t*b;  //��Ȼt�ǿ���a���Ƕεı���
}

//   �ݹ鱴�������� //�൱��һ��һ���ظ���t�ҵ㲢�һ���
cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    //��������
    if(control_points.size()==1) return control_points[0];
    // �½�һ�����飬�����µĿ��Ƶ㣬����ÿ�λ���� 1�����ٵ� 1ʱ��Ϊ����
	vector<Point2f>lerp_points;
    // ����ԭ���Ƶ����飬��ͨ�� lerp_v2f��������Ĺ�ʽ�����γɵĿ��Ƶ����������µ�����
	for(size_t i=1;i<control_points.size();i++)
	{
		lerp_points.push_back(lerp_v2f(control_points[i-1],control_points[i],t));
	}
	//�����ݹ�
	return recursive_bezier(lerp_points,t);
}

//�������� t
void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.   �ݹ鱴����
	for(double t=0.0;t<=1.0;t+=0.001)
	{
        // ����ÿһ�� t��������Ӧ�ı��������ߵĵ㣬������ͨ��������Ĺ�ʽ (1 - t) * a + t * b���ݹ���ã�
        // ʵ����Ҳ����ֱ���� n��չ��ʽ��ֱ����ã����� naive_bezier��������		
		auto point =recursive_bezier(control_points,t);
		window.at<Vec3b>(point.y, point.x)[2]=255; //����
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
