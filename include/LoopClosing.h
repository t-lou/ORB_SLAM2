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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include "KeyFrame.h"
#include "LocalMapping.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

#include "KeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <fstream>
#include <limits>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

namespace ORB_SLAM2
{

class Tracking;
class LocalMapping;
class KeyFrameDatabase;


class LoopClosing
{
public:

    typedef pair<set<KeyFrame*>,int> ConsistentGroup;
    typedef map<KeyFrame* const,g2o::Sim3,std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:

    LoopClosing(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

    ~LoopClosing()
    {
        using type_kf_id = decltype(KeyFrame::mnId);
        using type_pt_id = decltype(KeyFrame::mnId);
        struct KfInfo
        {
            std::string name;
            std::vector<float> tf;
            std::unordered_map<type_pt_id, std::pair<float, float>> points;
        };
        struct PtInto
        {
            std::vector<float> pos;
            std::queue<type_kf_id> seen_in;
        };
        struct Intr
        {
            float fx = -1.0f;
            float fy = -1.0f;
            float cx = -1.0f;
            float cy = -1.0f;
        } intr;

        std::unordered_map<type_kf_id, KfInfo> kfs;
        std::unordered_map<type_pt_id, PtInto> pts;

        auto visit = [&kfs,&pts,&intr](KeyFrame* frame) -> void
        {
            if (kfs.find(frame->mnId) != kfs.end())
            {
                std::cout << "frame with id " << frame->mnId << std::endl;
                return;
            }

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

            const std::vector<MapPoint*> pts_3d = frame->GetMapPointMatches();
            const std::vector<cv::KeyPoint> &pts_2d = frame->mvKeysUn;
            for (std::size_t i = 0u; i < pts_3d.size(); ++i)
            {
                if (pts_3d[i] != nullptr)
                {
                    if (pts.find(pts_3d[i]->mnId) == pts.end())
                    {
                        cv::Mat pos = pts_3d[i]->GetWorldPos();
                        pts[pts_3d[i]->mnId] = PtInto{std::vector<float>{
                            pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)}};
                    }
                    pts[pts_3d[i]->mnId].seen_in.push(frame->mnId);
                    kf_info.points[pts_3d[i]->mnId] = std::pair<float, float>(pts_2d[i].pt.x, pts_2d[i].pt.y);
                }
            }

            kfs[frame->mnId] = kf_info;

            if (intr.fx < 0.0f)
            {
                intr.fx = frame->fx;
                intr.fy = frame->fy;
                intr.cx = frame->cx;
                intr.cy = frame->cy;
            }
        };

        std::unordered_set<type_kf_id> visited;
        std::queue<KeyFrame*> to_visit;
        visited.insert(mpCurrentKF->mnId);
        to_visit.push(mpCurrentKF);

        while (!to_visit.empty())
        {
            std::queue<KeyFrame*> next;
            while (!to_visit.empty())
            {
                visit(to_visit.front());
                for (KeyFrame* frame: to_visit.front()->GetVectorCovisibleKeyFrames())
                {
                    if (frame != nullptr && visited.find(frame->mnId) == visited.end())
                    {
                        visited.insert(frame->mnId);
                        next.push(frame);
                    }
                }
                to_visit.pop();
            }
            to_visit = next;
        }

        YAML::Node root;
        for (const auto& kf: kfs)
        {
            root["key_frame"][kf.first]["name"] = kf.second.name;
            root["key_frame"][kf.first]["tf"] = kf.second.tf;
            for (const auto& pt : kf.second.points)
            {
                root["key_frame"][kf.first]["has"][pt.first] = std::vector<float>{
                    pt.second.first, pt.second.second};
            }
        }
        for (auto& point: pts)
        {
            root["mark"][point.first]["pos"] = point.second.pos;
            while (!point.second.seen_in.empty())
            {
                root["mark"][point.first]["in"].push_back(point.second.seen_in.front());
                point.second.seen_in.pop();
            }
        }

        root["intrinsics"]["fx"] = intr.fx;
        root["intrinsics"]["fy"] = intr.fy;
        root["intrinsics"]["cx"] = intr.cx;
        root["intrinsics"]["cy"] = intr.cy;

        std::ofstream out("/tmp/observations.yaml");
        out.precision(std::numeric_limits<float>::max_digits10);
        out << root;
    }

    void SetTracker(Tracking* pTracker);

    void SetLocalMapper(LocalMapping* pLocalMapper);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame *pKF);

    void RequestReset();

    // This function will run in a separate thread
    void RunGlobalBundleAdjustment(unsigned long nLoopKF);

    bool isRunningGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbRunningGBA;
    }
    bool isFinishedGBA(){
        unique_lock<std::mutex> lock(mMutexGBA);
        return mbFinishedGBA;
    }

    void RequestFinish();

    bool isFinished();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

protected:

    bool CheckNewKeyFrames();

    bool DetectLoop();

    bool ComputeSim3();

    void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

    void CorrectLoop();

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    Tracking* mpTracker;

    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBVocabulary;

    LocalMapping *mpLocalMapper;

    std::list<KeyFrame*> mlpLoopKeyFrameQueue;

    std::mutex mMutexLoopQueue;

    // Loop detector parameters
    float mnCovisibilityConsistencyTh;

    // Loop detector variables
    KeyFrame* mpCurrentKF;
    KeyFrame* mpMatchedKF;
    std::vector<ConsistentGroup> mvConsistentGroups;
    std::vector<KeyFrame*> mvpEnoughConsistentCandidates;
    std::vector<KeyFrame*> mvpCurrentConnectedKFs;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;

    long unsigned int mLastLoopKFid;

    // Variables related to Global Bundle Adjustment
    bool mbRunningGBA;
    bool mbFinishedGBA;
    bool mbStopGBA;
    std::mutex mMutexGBA;
    std::thread* mpThreadGBA;

    // Fix scale in the stereo/RGB-D case
    bool mbFixScale;


    int mnFullBAIdx;
};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
