/**
 * @file rgbd_tum.cc
 * @author guoqing (1337841346@qq.com)
 * @brief TUM RGBD 数据集上测试ORB-SLAM2
 * @version 0.1
 * @date 2019-02-16
 *
 * @copyright Copyright (c) 2019
 *
 */


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



#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>
#include<opencv2/core/core.hpp>

#include<System.h>
#include <librealsense2/rs.hpp>


using namespace std;


class RealsenseCam{
public:
    RealsenseCam();
    const vector<cv::Mat>* getFrames();

    int imageW;
    int imageH;

    vector<cv::Mat> frames;

private:
    rs2::pipeline pipe;
    rs2::align align_to_depth = rs2::align(RS2_STREAM_COLOR);
    rs2::frameset frameset;
    rs2::config cfg;

};

RealsenseCam::RealsenseCam() {
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_COLOR,640,480,RS2_FORMAT_ANY,60);
    cfg.enable_stream(RS2_STREAM_DEPTH,640,480,RS2_FORMAT_ANY,60);
    pipe.start(cfg);
    cout<<"RealsenseCamera is initializing ......";
    //culling first five frame.
    frameset = pipe.wait_for_frames();
    while(frameset == NULL)
        frameset = pipe.wait_for_frames();
    frameset = align_to_depth.process(frameset);

    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();

    imageW = depth.as<rs2::video_frame>().get_width();
    imageH = depth.as<rs2::video_frame>().get_height();
    cv::Mat depthCV{cv::Size(imageW,imageH),CV_16UC1,const_cast<void*>(depth.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat colorCV{cv::Size(imageW,imageH),CV_8UC3,const_cast<void*>(color.get_data()),cv::Mat::AUTO_STEP};

    frames.push_back(depthCV);
    frames.push_back(colorCV);

}

const vector<cv::Mat>* RealsenseCam::getFrames() {
    frameset = pipe.wait_for_frames();
    frameset = align_to_depth.process(frameset);
    auto depth = frameset.get_depth_frame();
    auto color = frameset.get_color_frame();
    cv::Mat depthCV{cv::Size(imageW,imageH),CV_16UC1,const_cast<void*>(depth.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat colorCV{cv::Size(imageW,imageH),CV_8UC3,const_cast<void*>(color.get_data()),cv::Mat::AUTO_STEP};

    frames.clear();
    frames.push_back(depthCV);
    frames.push_back(colorCV);
    return &frames;
}



int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    RealsenseCam realsenseCam;
    const vector<cv::Mat>* frames;
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
    cv::Mat imRGB, imD;

    while(1){
        frames = realsenseCam.getFrames();
        imD = (*frames)[0];
        imRGB = (*frames)[1];
        clock_t tframe = clock();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        //! 追踪
        SLAM.TrackRGBD(imRGB,imD,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

		cout<<"This ttrack use " << ttrack << "ms"<<endl;


        if(cv::waitKey(33)=='q')
            break;

    }
    SLAM.Shutdown();

    SLAM.SaveTrajectoryTUM("./CameraTrajectory2.txt");

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
