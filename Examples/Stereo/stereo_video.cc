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
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<librealsense2/rs.hpp>

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
    rs2::frameset frameset;
    rs2::config cfg;

};

RealsenseCam::RealsenseCam() {
    cfg.disable_all_streams();
    cfg.enable_stream(RS2_STREAM_INFRARED,640,480,RS2_FORMAT_Y8,60);
    pipe.start(cfg);
    cout<<"RealsenseCamera is initializing ......";
    //culling first five frame.
    frameset = pipe.wait_for_frames();
    while(frameset == NULL)
        frameset = pipe.wait_for_frames();

    auto infra1 = frameset.get_infrared_frame(0);
    auto infra2 = frameset.get_infrared_frame(1);   //qu chu IR guang

    imageW = infra1.as<rs2::video_frame>().get_width();
    imageH = infra1.as<rs2::video_frame>().get_height();

    cv::Mat ifra1CV{cv::Size(imageW,imageH),CV_8UC1,const_cast<void*>(infra1.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat ifra2CV{cv::Size(imageW,imageH),CV_8UC1,const_cast<void*>(infra2.get_data()),cv::Mat::AUTO_STEP};

    frames.push_back(ifra1CV);
    frames.push_back(ifra2CV);

}

const vector<cv::Mat>* RealsenseCam::getFrames() {
    frameset = pipe.wait_for_frames();
    auto infra1 = frameset.get_infrared_frame(0);
    auto infra2 = frameset.get_infrared_frame(1);
    cv::Mat ifra1CV{cv::Size(imageW,imageH),CV_8UC1,const_cast<void*>(infra1.get_data()),cv::Mat::AUTO_STEP};
    cv::Mat ifra2CV{cv::Size(imageW,imageH),CV_8UC1,const_cast<void*>(infra2.get_data()),cv::Mat::AUTO_STEP};

    frames.clear();
    frames.push_back(ifra1CV);
    frames.push_back(ifra2CV);
    return &frames;
}


int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings " << endl;
        return 1;
    }

    RealsenseCam realsenseCam;
    const vector<cv::Mat>* frames;
    cv::Mat imLeft, imRight;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    while(1){

        frames = realsenseCam.getFrames();
        imLeft = (*frames)[0];
        imRight = (*frames)[1];

        clock_t tframe = clock();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout<<"This ttrack use " << ttrack << "ms"<<endl;

        if(cv::waitKey(33)=='q')
            break;
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}
