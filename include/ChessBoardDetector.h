#ifndef CHESSBOARDDETECTOR_H
#define CHESSBOARDDETECTOR_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "BoostArchiver.h"
// #include "System.h"


namespace ORB_SLAM2
{

    class ChessBoardDetector
    {
    public:
        ChessBoardDetector();
        // ChessBoardDetector(const cv::Mat &img, cv::Size patternSize, float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
        ChessBoardDetector(const cv::Mat &img, int height, int width, float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs);
        bool isFound();
        bool findChessBoard();
        cv::Mat getCameraPose();
        cv::Mat calculateCameraPose();
        cv::Mat getImage();
        cv::Mat getCameraMatrix();
        cv::Mat getDistCoeffs();
        float getSquareSize();

        // Pose calculated in Frame Object
        void setFrameObjectPose(cv::Mat mTcw);
        cv::Mat getFrameObjectPose();
        // ------------------
        // friend class System;
    private:
        // serialize is recommended to be private
        friend class boost::serialization::access;
        template<class Archive>
        void serialize(Archive &ar, const unsigned int version);

    // private:
    public:
        cv::Mat image;
        // cv::Size patternSize; 
        int height, width;
        float squareSize;
        cv::Mat cameraMatrix; 
        cv::Mat distCoeffs;
        bool mbFindChessBoard;

        // 2D point corner detected from image by chess board pattern recognition
        std::vector<cv::Point2f> mvCorners;
        // 3D point of corner in chess board object coordinate
        std::vector<cv::Point3f> mvObjPoints;
        // camera pose in chess borad object coordinate
        cv::Mat mCameraPose;
        cv::Mat mTcw;
        // void initialChessBoardObjPoint(cv::Size patternSize, float squareSize);
        void initialChessBoardObjPoint(int height, int width, float squareSize);
    };
}

#endif