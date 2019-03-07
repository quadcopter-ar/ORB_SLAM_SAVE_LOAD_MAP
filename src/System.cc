/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

static bool has_suffix(const std::string &str, const std::string &suffix)
{
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer, bool is_save_map_):mSensor(sensor), is_save_map(is_save_map_), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),
        mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }

    cv::FileNode mapfilen = fsSettings["Map.mapfile"];
    bool bReuseMap = false;
    //cout<< typeid(mapfilen) <<endl;
    if (!mapfilen.empty())
    {
        mapfile = (string)mapfilen;
        cout << "file is not empty" << endl;
        // for(auto& temp:mapfilen)
        //     cout<<temp<<endl;
        //cout << &mapfile << endl;
    }

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else if(has_suffix(strVocFile, ".bin"))
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
    else
        bVocLoad = false;
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    scalingOnce = true;
    real_world_scale = 1;
    //Create KeyFrame Database
    //Create the Map
    if (!mapfile.empty() && LoadMap(mapfile))
    {
        bReuseMap = true;
    }
    else
    {
        mpKeyFrameDatabase = new KeyFrameDatabase(mpVocabulary);
        mpMap = new Map();
    }

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap, bReuseMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, bReuseMap);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile, bReuseMap);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    cout<< "shutting down, saving map"<<endl;
    if (is_save_map)
        SaveMap(mapfile);
    cout<< "map saved"<<endl;
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        cout << "Outside 1st while" << endl;
        while(!mpViewer->isFinished())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(5000));
        }
    }
     cout << "Outside 2nd while" << endl;
     //Debugging
     if(!mpLocalMapper->isFinished())
     {
        cout << "Local Mapper not finished" << endl;
     }
     else if(!mpLoopCloser->isFinished())
     {
        cout << "Loop Closing not finished" << endl;
     }
     else if(mpLoopCloser->isRunningGBA())
     {
        cout << "mpLoopCloser->isRunningGBA()" << endl;
     }
    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
    cout << "2nd while loop terminates" << endl;
    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    cout << "Shutdown function over" << endl;
    // if (is_save_map)
    //     SaveMap(mapfile);
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::SaveMap(const string &filename)
{
    std::ofstream out(filename, std::ios_base::binary);
    if (!out)
    {
        cerr << "Cannot Write to Mapfile: " << mapfile << std::endl;
        exit(-1);
    }
    cout << "Saving Mapfile: " << mapfile << std::flush;
    boost::archive::binary_oarchive oa(out, boost::archive::no_header);
    cout<<"Line 1"<<endl;
    oa << mpMap;
    cout<<"Line 2"<<endl;
    oa << mpKeyFrameDatabase;
    cout << " ...done" << std::endl;
    out.close();
}
bool System::LoadMap(const string &filename)
{
    // Added by Quadcopter
    /*When loading the map, we first calculate real world scale*/
    // Step 1. Load the two initialization frames
    ChessBoardDetector ref_frame_cbd, cur_frame_cbd;
    // Frame ref_frame, cur_frame;

    std::cout << "Loading stored objects" << std::endl;

    std::ifstream in_ref("ReferenceFrameCBD", std::ios_base::binary);
    if (!in_ref)
    {
        cerr << "Cannot Open file: ReferenceFrame. You need create it first!" << std::endl;
    }
    boost::archive::binary_iarchive ia_ref(in_ref, boost::archive::no_header);
    ia_ref >> ref_frame_cbd;
    in_ref.close();

    std::ifstream in_curr("CurrentFrameCBD", std::ios_base::binary);
    if (!in_curr)
    {
        cerr << "Cannot Open file: CurrentFrame. You need create it first!" << std::endl;
    }
    boost::archive::binary_iarchive ia_curr(in_curr, boost::archive::no_header);
    ia_curr >> cur_frame_cbd;
    in_curr.close();
//-----------------
    // std::ifstream in_ref2("ReferenceFrameObj", std::ios_base::binary);
    // if (!in_ref2)
    // {
    //     cerr << "Cannot Open file: ReferenceFrame. You need create it first!" << std::endl;
    // }
    // boost::archive::binary_iarchive ia_ref2(in_ref2, boost::archive::no_header);
    // ia_ref2 >> ref_frame;
    // in_ref2.close();

    // std::ifstream in_curr2("CurrentFrameObj", std::ios_base::binary);
    // if (!in_curr2)
    // {
    //     cerr << "Cannot Open file: CurrentFrame. You need create it first!" << std::endl;
    // }
    // boost::archive::binary_iarchive ia_curr2(in_curr2, boost::archive::no_header);
    // ia_curr2 >> cur_frame;
    // in_curr2.close();

    std::cout << "Stored objects loaded" << std::endl;

    std::cout << "ref_frame_cbd.width: " << ref_frame_cbd.width << std::endl;
    std::cout << "cur_frame_cbd.width: " << cur_frame_cbd.width << std::endl;
    std::cout << "ref_frame_cbd.height: " << ref_frame_cbd.height << std::endl;
    std::cout << "cur_frame_cbd.height: " << cur_frame_cbd.height << std::endl;
    std::cout << "ref_frame_cbd.squareSize: " << ref_frame_cbd.squareSize << std::endl;
    std::cout << "cur_frame_cbd.squareSize: " << cur_frame_cbd.squareSize << std::endl;

    // Step 2. Detect chessboard in the two frames
    ref_frame_cbd.findChessBoard();
    std::cout << "3..." << std::endl;
    cur_frame_cbd.findChessBoard();

    // Step 3. Calculate pose in the two frames
    cv::Mat refFramePose_world = ref_frame_cbd.getCameraPose(); // camera pose of the first frame relative to the chessboard
    cv::Mat curFramePose_world = cur_frame_cbd.getCameraPose(); // camera pose of the second frame relative to the chessboard



    // cv::Vec3d ref_to_curr_world(refFramePose_world[0][3] - curFramePose_world[0][3], 
    //                     refFramePose_world[1][3] - curFramePose_world[1][3],
    //                     refFramePose_world[2][3] - curFramePose_world[2][3]);
    cv::Vec3f ref_to_curr_world(refFramePose_world.at<float>(0, 3) - curFramePose_world.at<float>(0, 3), 
                        refFramePose_world.at<float>(1, 3) - curFramePose_world.at<float>(1, 3),
                        refFramePose_world.at<float>(2, 3) - curFramePose_world.at<float>(2, 3));
    float realWorldLen = sqrt(pow(ref_to_curr_world[0], 2) + pow(ref_to_curr_world[1], 2) + pow(ref_to_curr_world[2], 2));

    std::cout << "refFramePose_world" << refFramePose_world << std::endl;
    std::cout << "curFramePose_world" << curFramePose_world << std::endl;
    std::cout << "realWorldLen: " << realWorldLen << std::endl;

    cv::Mat refFramePose_slam = ref_frame_cbd.getFrameObjectPose(); // camera pose of the first frame relative to the chessboard
    cv::Mat curFramePose_slam = cur_frame_cbd.getFrameObjectPose(); // camera pose of the second frame relative to the chessboard

    // cv::Vec3d ref_to_curr_slam(refFramePose_slam[0][3] - curFramePose_slam[0][3], 
    //                     refFramePose_slam[1][3] - curFramePose_slam[1][3],
    //                     refFramePose_slam[2][3] - curFramePose_slam[2][3]);
    cv::Vec3f ref_to_curr_slam(refFramePose_slam.at<float>(0, 3) - curFramePose_slam.at<float>(0, 3), 
                        refFramePose_slam.at<float>(1, 3) - curFramePose_slam.at<float>(1, 3),
                        refFramePose_slam.at<float>(2, 3) - curFramePose_slam.at<float>(2, 3));
    float slamLen = sqrt(pow(ref_to_curr_slam[0], 2) + pow(ref_to_curr_slam[1], 2) + pow(ref_to_curr_slam[2], 2));

    std::cout << "refFramePose_slam" << refFramePose_slam << std::endl;
    std::cout << "curFramePose_slam" << curFramePose_slam << std::endl;
    std::cout << "slamLen: " << slamLen << std::endl;

    // Step 4. Calculate Scale
    // Write code to calculate scale
    // float scale = realWorldLen/slamLen;
    real_world_scale = realWorldLen/slamLen;
    // std::cout << "NEW SCALE: "<< scale << std::endl;

    // Quadcopterar..
    // Step 5. Load Map from mapfile
    std::ifstream in(filename, std::ios_base::binary);
    if (!in)
    {
        cerr << "Cannot Open Mapfile: " << mapfile << " , You need create it first!" << std::endl;
        return false;
    }
    cout << "Loading Mapfile: " << mapfile << std::flush;
    boost::archive::binary_iarchive ia(in, boost::archive::no_header);
    ia >> mpMap;
    ia >> mpKeyFrameDatabase;
    /*Map object loaded. Here we want to add the scaling code*/
    // // Step 6. Rescale the map points
    // if(scalingOnce)
    // {
    //     scalingOnce = false;
    //     std::vector<KeyFrame*> keyFrames =(mpMap -> GetAllKeyFrames());
    //     std::vector<MapPoint*> mapPoints =(mpMap -> GetAllMapPoints());

    //     // Rescale map points
    //     for(unsigned int i = 0 ; i < mapPoints.size(); i++)
    //     {
    //        MapPoint *pMP = mapPoints[i];
    //        pMP->SetWorldPos(pMP->GetWorldPos()*scale);
    //     }

    //     // Rescale keyframes
    //     for(unsigned int i = 0 ; i < keyFrames.size(); i++)
    //     {
    //         KeyFrame *kf = keyFrames[i];

    //         cv::Mat Tc2w = kf->GetPose();
    //         Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*scale;
    //         kf->SetPose(Tc2w);
    //     }
    // }
    
    mpKeyFrameDatabase->SetORBvocabulary(mpVocabulary);
    cout << " ...done" << std::endl;
    cout << "Map Reconstructing" << flush;
    vector<ORB_SLAM2::KeyFrame*> vpKFS = mpMap->GetAllKeyFrames();
    unsigned long mnFrameId = 0;
    for (auto it:vpKFS) {
        it->SetORBvocabulary(mpVocabulary);
        it->ComputeBoW();
        if (it->mnFrameId > mnFrameId)
            mnFrameId = it->mnFrameId;
    }
    Frame::nNextId = mnFrameId;
    cout << " ...done" << endl;
    in.close();
    return true;
}

} //namespace ORB_SLAM
