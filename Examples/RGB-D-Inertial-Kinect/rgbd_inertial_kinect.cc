/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <System.h>

#include <k4a/k4a.hpp>

using namespace std;

// A note to self:
// cv::Mat cImg = cv::Mat(img.get_height_pixels(), img.get_width_pixels(), CV_8UC4, img.get_buffer());

int main(int argc, char **argv)
{
    // === Argument Parsing ===
    if (argc < 3 || argc > 4)
    {
        cerr << endl
             << "Usage: ./rgbd_inertial_kinect path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4)
    {
        file_name = string(argv[argc - 1]);
    }

    // === Kinect Configuration ===
    // TODO: configure camera
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    device_config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    device_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    device_config.color_resolution = K4A_COLOR_RESOLUTION_720P;

    // === Start Kinect Data Streams ===
    k4a::device device = k4a::device::open(0);
    device.start_cameras(&device_config);
    device.start_imu();

    // === Setup ORBSlam3 ===
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_RGBD, true, 0, file_name);

    auto calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution);
    auto transformation = k4a::transformation(calibration);

    while (!SLAM.isShutDown())
    {
        // Sample RGB and Depth
        k4a::capture capture;
        if (!device.get_capture(&capture))
        {
            cerr << "[Kinect] Could not capture image" << endl;
            continue;
        };

        // Read from the buffered IMU samples in the Kinect SDK. For more information see the following:
        // * https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/group___functions_ga8e5913b3bb94a453c7143bbd6e399a0e.html#ga8e5913b3bb94a453c7143bbd6e399a0e
        // * https://docs.microsoft.com/en-us/azure/kinect-dk/retrieve-imu-samples#access-imu-samples
        // Using `i` sets an upper bound on IMU samples in case buffer grows too big. Unlikely to happen but being defensive here.
        vector<k4a_imu_sample_t> imu_samples;
        for (int i = 0; i < 1000; i++)
        {
            k4a_imu_sample_t imu_sample; // TODO: don't allocate
            auto res = device.get_imu_sample(&imu_sample, 0ms);
            if (!res)
            {
                // kinect buffer empty, we're done reading it
                break;
            }
            imu_samples.push_back(imu_sample);
        }
        // std::cout << "[Kinect] Number of IMU samples: " << imu_samples.size() << std::endl;
        // vector<k4a_imu_sample_t> last_n_imu_samples(imu_samples.end() - std::min<int>(imu_samples.size(), 50), imu_samples.end()); // TODO extract hardcoded value
        // std::cout << "[Kinect] Number of used IMU samples: " << last_n_imu_samples.size() << std::endl;

        // Convert RBG, Depth, and IMU data to what ORBSlam3 expects
        k4a::image colorImg = capture.get_color_image();
        k4a::image depthImg = transformation.depth_image_to_color_camera(capture.get_depth_image());

        if (!colorImg)
        {
            std::cerr << "[Kinect] colorImg not found! skipping..." << std::endl;
            continue;
        }
        if (!depthImg)
        {
            std::cerr << "[Kinect] depthImg not found! skipping..." << std::endl;
            continue;
        }

        // See here for more information:
        // * https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1482#issuecomment-763596308
        cv::Mat cvColorImg = cv::Mat(colorImg.get_height_pixels(), colorImg.get_width_pixels(), CV_8UC4, colorImg.get_buffer(), (size_t)colorImg.get_stride_bytes());
        cv::Mat cvDepthImg = cv::Mat(depthImg.get_height_pixels(), depthImg.get_width_pixels(), CV_8U, depthImg.get_buffer(), (size_t)depthImg.get_stride_bytes());

        // Commented out code for debugging.
        // cv::imshow("cvColorImg", cvColorImg);
        // cv::imshow("cvDepthImg", cvDepthImg);

        vector<ORB_SLAM3::IMU::Point> imu_points;
        for (auto const &sample : imu_samples)
        {
            auto acc = sample.acc_sample.xyz;
            auto gyro = sample.gyro_sample.xyz;
            ORB_SLAM3::IMU::Point point(acc.x, acc.y, acc.z,
                                        gyro.x, gyro.y, gyro.z,
                                        sample.acc_timestamp_usec * 1e-6); // TODO: timestamp in seconds?
            imu_points.push_back(point);
        }

        std::chrono::microseconds timestamp = colorImg.get_device_timestamp();
        SLAM.TrackRGBD(cvColorImg, cvDepthImg, timestamp.count() * 1e-6, imu_points);
    }

    return 0;
}