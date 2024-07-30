#include "OccMapTransform.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

void displayTransformedPoint(const Mat& map, const Point& transformed_point);

void OccupancyGridParam::GetOccupancyGridParam(nav_msgs::OccupancyGrid OccGrid)
{
    // Get parameter
    resolution = OccGrid.info.resolution;
    height = OccGrid.info.height;
    width = OccGrid.info.width;
    x = OccGrid.info.origin.position.x;
    y = OccGrid.info.origin.position.y;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = OccGrid.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    theta = yaw;

    // Calculate R, t
    R = Mat::zeros(2,2, CV_64FC1);
    R.at<double>(0, 0) = resolution * cos(theta);
    R.at<double>(0, 1) = resolution * sin(-theta);
    R.at<double>(1, 0) = resolution * sin(theta);
    R.at<double>(1, 1) = resolution * cos(theta);
    t = Mat(Vec2d(x, y), CV_64FC1);
    ROS_ERROR("OccupancyGridParam: resolution=%f, width=%d, height=%d, x=%f, y=%f, theta=%f",
             resolution, width, height, x, y, theta);
    // Occupancy Grid 맵 데이터 저장
    Map = cv::Mat(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            int OccProb = OccGrid.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            Map.at<uchar>(height - i - 1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

}

void OccupancyGridParam::Image2MapTransform(Point& src_point, Point2d& dst_point)
{
    // Upside down
    Mat P_src = Mat(Vec2d(src_point.x, height - 1 - src_point.y), CV_64FC1);
    // Rotate and translate
    Mat P_dst = R * P_src + t;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

void OccupancyGridParam::Map2ImageTransform(Point2d& src_point, Point& dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    // Rotate and translate
    Mat P_dst = R.inv() * (P_src - t);
    // Upside down
    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = height - 1 - round(P_dst.at<double>(1, 0));

    // ROS_ERROR("set: src=(%.2f, %.2f) -> dst=(%d, %d)", src_point.x, src_point.y, dst_point.x, dst_point.y);
    // displayTransformedPoint(Map, dst_point);

}

void displayTransformedPoint(const Mat& map, const Point& transformed_point) {
    Mat display_map;

    cvtColor(map, display_map, cv::COLOR_GRAY2BGR); // 그레이스케일 맵을 컬러로 변환
    circle(display_map, transformed_point, 5, Scalar(0, 0, 255), -1); // 빨간색 점 그리기

    // 이미지 창에 띄우기
    namedWindow("Transformed Point", WINDOW_AUTOSIZE);
    imshow("Transformed Point", display_map);
    waitKey(0); // 키 입력 대기
}
