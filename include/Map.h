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

#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "SystemSetting.h"

#include <set>

#include <mutex>



namespace ORB_SLAM2
{

class MapPoint; //地图点
class KeyFrame; //关键帧
class KeyFrameDatabase;
class Map
{
public:
    Map();

    void AddKeyFrame(KeyFrame* pKF); //添加关键帧
    void AddMapPoint(MapPoint* pMP); //添加地图点
    void EraseMapPoint(MapPoint* pMP); //清除地图点
    void EraseKeyFrame(KeyFrame* pKF); //清除关键帧
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs); //设置参考地图点
    void InformNewBigChange(); //重大变化提醒
    int GetLastBigChangeIdx(); //上一次重大变化发生时的索引

    std::vector<KeyFrame*> GetAllKeyFrames(); //获得所有关键帧
    std::vector<MapPoint*> GetAllMapPoints(); //获得所有地图点
    std::vector<MapPoint*> GetReferenceMapPoints(); //获得参考地图点

    long unsigned int MapPointsInMap(); //地图中地图点的数量
    long unsigned  KeyFramesInMap(); //地图中关键帧的数量

    long unsigned int GetMaxKFid(); //获得最大关键帧索引

    void clear(); //清除所有

    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;

    void Save(const string& filename);
    void Load(const string &filename,SystemSetting* mySystemSetting, KeyFrameDatabase* kfd);
    MapPoint* LoadMapPoint(ifstream &f);
    KeyFrame* LoadKeyFrame(ifstream &f,SystemSetting* mySystemSetting);

protected:
    std::set<MapPoint*> mspMapPoints; //保存地图点
    std::set<KeyFrame*> mspKeyFrames; //保存关键帧

    std::vector<MapPoint*> mvpReferenceMapPoints; //参考地图点
    std::map<MapPoint*,unsigned long int> mmpnMapPointsIdx; //地图点和索引

    long unsigned int mnMaxKFid; //最大关键帧索引

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

    void SaveMapPoint(ofstream& f, MapPoint* mp);
    void SaveKeyFrame(ofstream& f, KeyFrame* kf);
    void GetMapPointsIdx();
};

} //namespace ORB_SLAM

#endif // MAP_H
