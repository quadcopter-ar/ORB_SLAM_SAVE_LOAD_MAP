#ifndef CHESSBOARDDETECTOR_H
#define CHESSBOARDDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>



namespace ORB_SLAM2
{

class ChessBoardDetector
{
public:
    ChessBoardDetector();
    ChessBoardDetector(const cv::Mat &img, cv::Size patternSize, float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
    bool isFound();
    cv::Mat getCameraPose();
private:
    bool mbFindChessBoard;
    // 2D point corner detected from image by chess board pattern recognition
    std::vector<cv::Point2f> mvCorners;
    // 3D point of corner in chess borad object coordinate
    std::vector<cv::Point3f> mvObjPoints;
    // camera pose in chess borad object coordinate
    cv::Mat mCameraPose;

    void initialChessBoardObjPoint(cv::Size patternSize, float squareSize);
};
}

#endif