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
        std::unordered_set<long unsigned int> visited;
        YAML::Node node;
        auto visit = [&visited, &node](KeyFrame* frame) -> void
        {
            if (frame != nullptr && visited.find(frame->mnId) == visited.end())
            {
                visited.insert(frame->mnId);

                cv::Mat tf = frame->GetPose();
                std::vector<float> tf_vec(7);
                tf_vec[0] = tf.at<float>(0, 3);
                tf_vec[1] = tf.at<float>(1, 3);
                tf_vec[2] = tf.at<float>(2, 3);
                Eigen::Matrix3f rot;
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        rot(i, j) = tf.at<float>(i, j);
                    }
                }
                Eigen::Quaternionf q(rot);
                tf_vec[3] = q.x();
                tf_vec[4] = q.y();
                tf_vec[5] = q.z();
                tf_vec[6] = q.w();

                bool changed = false;
                YAML::Node point_node;

                for (auto point: frame->GetMapPointMatches())
                {
                    if (point != nullptr)
                    {
                        cv::Mat pos = point->GetWorldPos();
                        point_node[point->mnId] = std::vector<float>{pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)};
                        changed = true;
                    }
                }
                if (changed)
                {
                    YAML::Node frame_node;
                    frame_node["tf"] = tf_vec;
                    frame_node["pt"] = point_node;
                    node[frame->mnId] = frame_node;
                }
            }
        };

        std::queue<KeyFrame*> to_visit;
        to_visit.push(mpCurrentKF);

        while (!to_visit.empty())
        {
            std::queue<KeyFrame*> next;
            while (!to_visit.empty())
            {
                const std::vector<KeyFrame*>& connected = to_visit.front()->GetVectorCovisibleKeyFrames();
                std::for_each(connected.begin(), connected.end(), visit);
                to_visit.pop();
            }
            to_visit = next;
        }

        std::ofstream o("/tmp/observations.yaml");
        o.precision(std::numeric_limits<float>::max_digits10);
        o << node;
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
