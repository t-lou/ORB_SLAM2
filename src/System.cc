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
#include <unistd.h>

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
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


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = std::make_unique<ORBVocabulary>();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = std::make_unique<KeyFrameDatabase>(*mpVocabulary);

    //Create the Map
    mpMap = std::make_unique<Map>();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = std::make_unique<FrameDrawer>(&(*mpMap));
    mpMapDrawer = std::make_unique<MapDrawer>(&(*mpMap), strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = std::make_unique<Tracking>(this, &(*mpVocabulary), &(*mpFrameDrawer), &(*mpMapDrawer),
                                           &(*mpMap), &(*mpKeyFrameDatabase), strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = std::make_unique<LocalMapping>(&(*mpMap), mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,&(*mpLocalMapper));

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = std::make_unique<LoopClosing>(&(*mpMap), &(*mpKeyFrameDatabase), &(*mpVocabulary), mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, &(*mpLoopCloser));

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = std::make_unique<Viewer>(this, &(*mpFrameDrawer),&(*mpMapDrawer),&(*mpTracker),strSettingsFile);
        mptViewer = new thread(&Viewer::Run, &(*mpViewer));
        mpTracker->SetViewer(&(*mpViewer));
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(&(*mpLocalMapper));
    mpTracker->SetLoopClosing(&(*mpLoopCloser));

    mpLocalMapper->SetTracker(&(*mpTracker));
    mpLocalMapper->SetLoopCloser(&(*mpLoopCloser));

    mpLoopCloser->SetTracker(&(*mpTracker));
    mpLoopCloser->SetLocalMapper(&(*mpLocalMapper));
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
                usleep(1000);
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
                usleep(1000);
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
                usleep(1000);
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

    mpTracker->DefineFrameName(msNextFrameName);
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
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
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

void System::ExportPose(const std::string& out_path)
{

    using type_kf_id = decltype(KeyFrame::mnId);
    using type_pt_id = decltype(KeyFrame::mnId);
    struct KfInfo
    {
        std::string name;
        std::vector<float> tf;
        std::unordered_map<type_pt_id, std::pair<float, float>> points;
        std::unordered_map<type_pt_id, std::pair<float, float>> points_orig;
    };
    struct PtInfo
    {
        std::vector<float> pos;
        std::unordered_set<type_kf_id> seen_in;
        type_kf_id ref_id;
        std::unordered_set<type_kf_id> seen_in_by_MapPoint;
    };

    struct ExportData
    {
        struct Intr
        {
            float fx = -1.0f;
            float fy = -1.0f;
            float cx = -1.0f;
            float cy = -1.0f;
        } intr;

        std::unordered_map<type_kf_id, KfInfo> kfs;

        std::unordered_map<type_pt_id, PtInfo> pts;
    } export_data;

    auto create_kf_itself = [](KeyFrame* frame) -> KfInfo
    {
        KfInfo kf_info;

        kf_info.name = frame->GetName();

        kf_info.tf.resize(7);
        cv::Mat tf = frame->GetPose();
        kf_info.tf[0] = tf.at<float>(0, 3);
        kf_info.tf[1] = tf.at<float>(1, 3);
        kf_info.tf[2] = tf.at<float>(2, 3);
        Eigen::Matrix3f rot;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                rot(i, j) = tf.at<float>(i, j);
            }
        }
        Eigen::Quaternionf q(rot);
        kf_info.tf[3] = q.x();
        kf_info.tf[4] = q.y();
        kf_info.tf[5] = q.z();
        kf_info.tf[6] = q.w();
        return kf_info;
    };

    auto connect_kf_pt = [&export_data](KeyFrame* frame, KfInfo &kf_info) -> void
    {
        const std::vector<MapPoint*> pts_3d = frame->GetMapPointMatches();
        const std::vector<cv::KeyPoint> &pts_2d = frame->mvKeysUn;
        const std::vector<cv::KeyPoint> &pts_2d_orig = frame->mvKeys;
        for (std::size_t i = 0u; i < pts_3d.size(); ++i)
        {
            if (pts_3d[i] != nullptr && !pts_3d[i]->isBad())
            {
                export_data.pts[pts_3d[i]->mnId].seen_in.insert(frame->mnId);
                kf_info.points[pts_3d[i]->mnId] = std::pair<float, float>(pts_2d[i].pt.x, pts_2d[i].pt.y);
                kf_info.points_orig[pts_3d[i]->mnId] = std::pair<float, float>(pts_2d_orig[i].pt.x, pts_2d_orig[i].pt.y);
            }
        }
    };
    // end of callback and data container 

    for (MapPoint* mark : mpMap->GetAllMapPoints())
    {
        if (mark != nullptr and !mark->isBad())
        {
            if (export_data.pts.find(mark->mnId) == export_data.pts.end())
            {
                cv::Mat pos = mark->GetWorldPos();
                export_data.pts[mark->mnId] = PtInfo{
                    std::vector<float>{pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)},
                    std::unordered_set<type_kf_id>{}, // filled in connect_kf_pt
                    mark->GetReferenceKeyFrame()->mnId,
                    mark->GetObsKfIds()
                };
            }
        }
    }
    for (KeyFrame* kf : mpMap->GetAllKeyFrames())
    {
        if (kf != nullptr and !kf->isBad())
        {
            KfInfo kf_info = create_kf_itself(kf);
            connect_kf_pt(kf, kf_info);
            export_data.kfs[kf->mnId] = kf_info;

            if (export_data.intr.fx < 0.0f)
            {
                export_data.intr.fx = kf->fx;
                export_data.intr.fy = kf->fy;
                export_data.intr.cx = kf->cx;
                export_data.intr.cy = kf->cy;
            }
        }
    }
    int counter{0};
    for (const auto & pt : export_data.pts){
        if(pt.second.seen_in_by_MapPoint!=pt.second.seen_in){
            std::cout << "inconsistent observations of landmark: " << pt.first << std::endl;
            counter++;
        }
    }
    std::cout << "Summary:" << counter << "/" << export_data.pts.size() << " are inconsistent in total." << std::endl;

    YAML::Node root;
    for (const auto& kf: export_data.kfs)
    {
        root["key_frame"][kf.first]["name"] = kf.second.name;
        root["key_frame"][kf.first]["tf"] = kf.second.tf;
        for (const auto& pt : kf.second.points)
        {
            root["key_frame"][kf.first]["has"][pt.first] = std::vector<float>{
                pt.second.first, pt.second.second};
        }
        for (const auto& pt : kf.second.points_orig)
        {
            root["key_frame"][kf.first]["px"][pt.first] = std::vector<float>{
                pt.second.first, pt.second.second};
        }
    }
    for (auto& point: export_data.pts)
    {
        root["mark"][point.first]["pos"] = point.second.pos;
        root["mark"][point.first]["ref"] = point.second.ref_id;
        root["mark"][point.first]["in"] = std::vector<type_kf_id>(
            point.second.seen_in.begin(), point.second.seen_in.end());
        root["mark"][point.first]["in_by_MapPoint"] = std::vector<type_kf_id>(
            point.second.seen_in_by_MapPoint.begin(), point.second.seen_in_by_MapPoint.end());
    }

    root["intrinsics"]["fx"] = export_data.intr.fx;
    root["intrinsics"]["fy"] = export_data.intr.fy;
    root["intrinsics"]["cx"] = export_data.intr.cx;
    root["intrinsics"]["cy"] = export_data.intr.cy;

    std::ofstream out(out_path);
    out.precision(std::numeric_limits<float>::max_digits10);
    out << root;
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

} //namespace ORB_SLAM
