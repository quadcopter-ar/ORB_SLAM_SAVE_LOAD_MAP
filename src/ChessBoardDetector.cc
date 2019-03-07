#include "ChessBoardDetector.h"


namespace ORB_SLAM2
{

    ChessBoardDetector::ChessBoardDetector(){}

    // ChessBoardDetector::ChessBoardDetector(const cv::Mat &img, cv::Size patternSize, float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
    ChessBoardDetector::ChessBoardDetector(const cv::Mat &img, int height, int width, float squareSize, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
    {
        this->cameraMatrix = cameraMatrix;
        this->distCoeffs = distCoeffs;
        // this->patternSize = patternSize;
        this->height = height;
        this->width = width; 
        this->squareSize = squareSize;
        this->image = img;
    }

    bool ChessBoardDetector::findChessBoard()
    {
        //Detects corners in 2D and stores them in mvCorners vector
        cv::Size patternSize(width, height); 
        mbFindChessBoard = cv::findChessboardCorners(image, patternSize, mvCorners,
        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
        + cv::CALIB_CB_FAST_CHECK);

        std::cout << "num of corner detected " << mvCorners.size() << std::endl;

        if(mbFindChessBoard)    
        {
            //Defining termiation criteria for the iterative algorithm
            cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1);
            //cv::cornerSubPix(img, mvCorners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
        }
        std::cout << "num of corner refined " << mvCorners.size() << std::endl;

        calculateCameraPose();

        return mbFindChessBoard;
    }

    bool ChessBoardDetector::isFound()
    {
        return mbFindChessBoard;
    }

    cv::Mat ChessBoardDetector::getCameraPose()
    {
        return mCameraPose;
    }

    //void ChessBoardDetector::initialChessBoardObjPoint(cv::Size patternSize, float squareSize)
    void ChessBoardDetector::initialChessBoardObjPoint(int height, int width, float squareSize)
    {
        // empty object point set
        mvObjPoints.clear();

        // for( int i = 0; i < patternSize.height; i++ )
        for( int i = 0; i < height; i++ )
        {
            // for( int j = 0; j < patternSize.width; j++ )
            for( int j = 0; j < width; j++ )
            {
            mvObjPoints.push_back(cv::Point3f(float(j*squareSize), float(i*squareSize), 0));
            }
        }
    }

    cv::Mat ChessBoardDetector::calculateCameraPose()
    {
        // initialize chess board corner in object coordinate
        initialChessBoardObjPoint(height, width, squareSize);
        std::cout << "num of chessboard point " << mvObjPoints.size() << std::endl;
        // calculate camera pose in chess board coordinate
        cv::Mat rvec(3,1,cv::DataType<double>::type);
        cv::Mat tvec(3,1,cv::DataType<double>::type);
        
        cv::solvePnP(cv::Mat(mvObjPoints), cv::Mat(mvCorners), cameraMatrix, distCoeffs, rvec, tvec);
        std::cout << "1..." << std::endl;
        // converts rotation vector to rotation matrix
        cv::Mat rmat(3,3, cv::DataType<double>::type);
        cv::Rodrigues(rvec, rmat);

        // copy rotation matrix and translate vector to pose matrix
        mCameraPose = cv::Mat::eye(4,4,CV_32F);
        rmat.copyTo(mCameraPose.rowRange(0,3).colRange(0,3));
        tvec.copyTo(mCameraPose.rowRange(0,3).col(3));
        std::cout << "2..." << std::endl;
        return mCameraPose;
    }

    cv::Mat ChessBoardDetector::getImage()
    {
        return this->image;
    }

    cv::Mat ChessBoardDetector::getCameraMatrix()
    {
        return this->cameraMatrix;
    }

    cv::Mat ChessBoardDetector::getDistCoeffs()
    {
        return this->distCoeffs;
    }

    float ChessBoardDetector::getSquareSize()
    {
        return squareSize;
    }

    void ChessBoardDetector::setFrameObjectPose(cv::Mat mTcw)
    {
        this->mTcw = mTcw;
    }

    cv::Mat ChessBoardDetector::getFrameObjectPose()
    {
        return mTcw;
    }

    template<class Archive>
    void ChessBoardDetector::serialize(Archive &ar, const unsigned int version)
    {
        ar & image;
        //cv::Size patternSize; 
        ar & height;
        ar & width;
        ar & squareSize;
        ar & cameraMatrix;
        ar & distCoeffs;
        ar & mTcw;
    }
    template void ChessBoardDetector::serialize(boost::archive::binary_iarchive&, const unsigned int);
    template void ChessBoardDetector::serialize(boost::archive::binary_oarchive&, const unsigned int);

}