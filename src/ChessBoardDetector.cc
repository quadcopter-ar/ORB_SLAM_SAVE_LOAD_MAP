#include "ChessBoardDetector.h"

namespace ORB_SLAM2
{

ChessBoardDetector::ChessBoardDetector()
{}

ChessBoardDetector::ChessBoardDetector(const cv::Mat &img, cv::Size patternSize, float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    mFindChessBoard = cv::findChessboardCorners(img, patternSize, corners,
    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
    + CALIB_CB_FAST_CHECK);
    if(mFindChessBoard)
        cv::cornerSubPix(img, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    
    // initialize chess board corner in object coordinate
    initialChessBoardObjPoint(patternSize, squareSize);

    // calculate camera pose in chess board coordinate
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    cv::solvePnP(Mat(mvObjPoints), Mat(corners), cameraMatrix, distCoeffs, rvec, tvec);

    // converts rotation vector to rotation matrix
    cv::Mat rmat(3,3, cv::DataType<double>::type);
    cv::Rodrigues(rvec, rmat);

    // copy rotation matrix and translate vector to pose matrix
    mCameraPose = cv::Mat::eye(4,4,CV_32F);
    rmat.copyTo(mCameraPose.rowRange(0,3).colRange(0,3));
    tvec.copyTo(mCameraPose.rowRange(0,3).col(3));
}

bool ChessBoardDetector::isFound()
{
    return mbFindChessBoard;
}

cv::Mat ChessBoardDetector::getCameraPose()
{
    return mCameraPose;
}

void initialChessBoardObjPoint(cv::Size patternSize, float squareSize)
{
    // empty object point set
    mvObjPoints.clear();

    for( int i = 0; i < patternSize.height; i++ )
    {
        for( int j = 0; j < patternSize.width; j++ )
        {
        mvObjPoints.push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));
        }
    }
}