/*******************************************************
 * Copyright (C) 2025, Trustworthy AI and Autonomous Systems (TAS) Lab, Hong Kong Polytechnic University
 * 
 * This file is part of GraphGNSSlib_LEO.
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Yixin GAO (yixin.gao@connect.polyu.hk)
 * 
 * Main fucntions: Combine GNSS and LEO Doppler element and Pseudorange, carrier phase raw msg into one topic
 * input: pseudorange, Doppler element from GPS/BeiDou/LEO.
 * output: GNSS_LEO_PsrCarRov1 topic 
 * Date: 2025/03/18
 *******************************************************/

// std inputs and outputs, fstream
#include <iostream>
#include <string>  
#include <fstream>
#include<sstream>
#include <stdlib.h>
#include <iomanip>

// math
#include <math.h>
//time 
#include <time.h>
//algorithm 
#include <algorithm>

// google eigen
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include<Eigen/Core>

// google implements commandline flags processing.
#include <gflags/gflags.h>
// google loging tools
#include <glog/logging.h>
// ros
#include <ros/ros.h>
/* Reference from NovAtel GNSS/INS */
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include "gnss_tools.h"
#include <nlosexclusion/GNSS_Raw.h>
#include <nlosexclusion/GNSS_Raw_Array.h>
#include <geometry_msgs/Point32.h>
#include <stdio.h>
#include <queue>
#include <map>
#include <queue>
#include <mutex>
#include <thread>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "../tic_toc.h"
// allign 
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/NavSatFix.h>

#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <novatel_msgs/BESTPOS.h> // novatel_msgs/INSPVAX

// rtklib
#include <stdarg.h>
#include "../../RTKLIB/src/rtklib.h"

class gnss_leo_msg_combination
{
    ros::NodeHandle nh;

    // ros::Publisher pub_WLSENU, pub_FGOENU, pub_global_path, pubStationGNSSRaw;
    ros::Publisher pubRoverGNSSRaw,pubStationGNSSRaw;
    /* ros subscriber */
    std::map<double, nlosexclusion::GNSS_Raw_Array> gnss_leo_raw_map;

    GNSS_Tools m_GNSS_Tools; // utilities

    /* subscriber */
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> rover_gnss_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> rover_leo_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> station_gnss_raw_array_sub;
    std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> station_leo_raw_array_sub;
    // std::unique_ptr<message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>> station_gnss_raw_array_sub;

    typedef message_filters::sync_policies::ApproximateTime<nlosexclusion::GNSS_Raw_Array, nlosexclusion::GNSS_Raw_Array> MySyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> syncRoverGNSSRaw;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> syncStationGNSSRaw;
    //std::unique_ptr<message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nlosexclusion::GNSS_Raw_Array>> syncRoverGNSSRaw;
    //std::unique_ptr<message_filters::TimeSynchronizer<nlosexclusion::GNSS_Raw_Array, nlosexclusion::GNSS_Raw_Array>> syncStationGNSSRaw;

    /* thread lock for data safe */
    std::mutex m_gnss_raw_mux;

    int gnss_frame = 0;
    int curGPSSec = 0;

    bool hasNewData = false;

    /* thread for data processing */
    // std::thread publishGNSSTopicThread;

    /* parameters to be get */
    int startGPSSec = 0;
    int endGPSSec   = 456900; 

    Eigen::Matrix<double, 3,1> ENU_ref;


    bool finishGNSSReader = false;

public:
    gnss_leo_msg_combination(ros::NodeHandle& nh)
    {      
        
        /* subscriber of three topics  */
        rover_gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarRov1", 10000));
        rover_leo_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/leo_raw_publisher_node/LEOPsrCarRov1", 10000));
        station_gnss_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/gnss_preprocessor_node/GNSSPsrCarStation1", 10000));
        station_leo_raw_array_sub.reset(new message_filters::Subscriber<nlosexclusion::GNSS_Raw_Array>(nh, "/leo_raw_publisher_node/LEOPsrCarStation1", 10000));
        
        /* publish the raw gnss data*/
        pubRoverGNSSRaw = nh.advertise<nlosexclusion::GNSS_Raw_Array>("/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov", 100); //

        /* publish the raw gnss data from station*/ 
        pubStationGNSSRaw = nh.advertise<nlosexclusion::GNSS_Raw_Array>("/gnss_leo_msg_combination_node/GNSS_LEO_PsrCarStation", 100); // 
        
        syncRoverGNSSRaw.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10000), *rover_gnss_raw_array_sub, *rover_leo_raw_array_sub));
        syncRoverGNSSRaw->registerCallback(boost::bind(&gnss_leo_msg_combination::gnss_leo_rover_raw_msg_callback,this, _1, _2));
        syncStationGNSSRaw.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10000), *station_gnss_raw_array_sub, *station_leo_raw_array_sub));
        syncStationGNSSRaw->registerCallback(boost::bind(&gnss_leo_msg_combination::gnss_leo_station_raw_msg_callback,this, _1, _2));



        /* reference point for ENU calculation */
        ENU_ref<< ref_lon, ref_lat, ref_alt;

        /* get parameters */
        nh.param("startGPSSec",   startGPSSec, 2);
        nh.param("endGPSSec",     endGPSSec, 2);
        // nh.param("soltype",soltype, 2);

    }

    /* check the valid epoch based on gps time span*/
    bool checkValidEpoch(double gps_sec)
    {
        if((gps_sec >= start_gps_sec) && (gps_sec <=end_gps_sec))
        {
            return true;
        }
        else return false;
    }

   
    /**
     * @brief callback function for gnss msg from leo and gnss from Rover
     * @param gnss_msg gnss raw msg from leo
     * @param leo_msg gnss raw msg from gnss
     * @return void
     * @ Similar function with gnss_leo_station_raw_msg_callback, except publish topic name
     */
   void gnss_leo_rover_raw_msg_callback(const boost::shared_ptr<const nlosexclusion::GNSS_Raw_Array>& gnss_msg,
    const boost::shared_ptr<const nlosexclusion::GNSS_Raw_Array>& leo_msg)
    {
        std::lock_guard<std::mutex> lock(m_gnss_raw_mux);
    
            
        if(finishGNSSReader) 
        {
            //m_gnss_raw_mux.unlock();
            return;
        }

        hasNewData = true;
        gnss_frame++;
        double time0 = gnss_msg->GNSS_Raws[0].GNSS_time;
        double time1 = leo_msg->GNSS_Raws[0].GNSS_time;

        curGPSSec = gnss_msg->GNSS_Raws[0].GNSS_time;
        
        std::cout<<"gnss time1: " <<time0 <<"; gnss time 2: "<<time1<<std::endl; 
        std::cout<<"gnss_msg size:"<<gnss_msg->GNSS_Raws.size()<<", "<<leo_msg->GNSS_Raws.size()<<std::endl;

        /* Combine LEO and GNSS msg to one topic*/
        nlosexclusion::GNSS_Raw_Array gnss_leo_msg = *gnss_msg;
        gnss_leo_msg.GNSS_Raws.insert(gnss_leo_msg.GNSS_Raws.end(), leo_msg->GNSS_Raws.begin(), leo_msg->GNSS_Raws.end());
        for (int i=0; i<gnss_leo_msg.GNSS_Raws.size();i++)
        {
            gnss_leo_msg.GNSS_Raws[i].total_sv = gnss_leo_msg.GNSS_Raws.size();
        }
        std::cout<<" GNSS_LEO Raw msg size is" << gnss_leo_msg.GNSS_Raws.size() << std::endl;
        gnss_leo_msg.header.stamp = gnss_msg->header.stamp;
            
        /* save the data */
        gnss_leo_raw_map[curGPSSec] = gnss_leo_msg;
        
        pubRoverGNSSRaw.publish(gnss_leo_msg);
        if(curGPSSec>end_gps_sec)
        {
            finishGNSSReader = true;
            std::cout<< " you can play the bag file now!  " << gnss_leo_raw_map.size()<<std::endl;
        }
        
        /* release the lock */
        m_gnss_raw_mux.unlock();
    }

    /**
     * @brief callback function for gnss msg from leo and gnss from station
     * @param gnss_msg gnss raw msg from leo
     * @param leo_msg gnss raw msg from station
     * @return void
     * @ Similar function with gnss_leo_rover_raw_msg_callback, except publish topic name
     */
    void gnss_leo_station_raw_msg_callback(const boost::shared_ptr<const nlosexclusion::GNSS_Raw_Array>& gnss_msg,
        const boost::shared_ptr<const nlosexclusion::GNSS_Raw_Array>& leo_msg)
        {
            std::lock_guard<std::mutex> lock(m_gnss_raw_mux);
    
            
            if(finishGNSSReader) 
            {
                //m_gnss_raw_mux.unlock();
                return;
            }
    
            hasNewData = true;
            gnss_frame++;
            double time0 = gnss_msg->GNSS_Raws[0].GNSS_time;
            double time1 = leo_msg->GNSS_Raws[0].GNSS_time;
    
            curGPSSec = gnss_msg->GNSS_Raws[0].GNSS_time;
            
            std::cout<<"gnss time1: " <<time0 <<"; gnss time 2: "<<time1<<std::endl; 
            std::cout<<"gnss_msg size:"<<gnss_msg->GNSS_Raws.size()<<", "<<leo_msg->GNSS_Raws.size()<<std::endl;
    
            /* Combine LEO and GNSS msg to one topic*/
            nlosexclusion::GNSS_Raw_Array gnss_leo_msg = *gnss_msg;
            gnss_leo_msg.GNSS_Raws.insert(gnss_leo_msg.GNSS_Raws.end(), leo_msg->GNSS_Raws.begin(), leo_msg->GNSS_Raws.end());
            for (int i=0; i<gnss_leo_msg.GNSS_Raws.size();i++)
            {
                gnss_leo_msg.GNSS_Raws[i].total_sv = gnss_leo_msg.GNSS_Raws.size();
            }
            std::cout<<" GNSS_LEO Raw msg size is" << gnss_leo_msg.GNSS_Raws.size() << std::endl;
            gnss_leo_msg.header.stamp = gnss_msg->header.stamp;
                
            /* save the data */
            gnss_leo_raw_map[curGPSSec] = gnss_leo_msg;
            
            pubStationGNSSRaw.publish(gnss_leo_msg);
            if(curGPSSec>end_gps_sec)
            {
                finishGNSSReader = true;
                std::cout<< " you can play the bag file now!  " << gnss_leo_raw_map.size()<<std::endl;
            }
            
            /* release the lock */
            m_gnss_raw_mux.unlock();
        }

    ~gnss_leo_msg_combination()
    {
    }
   
};

int main(int argc, char **argv)
{
    FLAGS_logtostderr = 1;  // output to console
    google::InitGoogleLogging(argv[0]); // init the google logging
    google::ParseCommandLineFlags(&argc, &argv, true); // parseCommandLineFlags 
    ros::init(argc, argv, "gnss_leo_msg_combination_node"); 
    ros::NodeHandle nh;
    ROS_INFO("\033[1;32m----> gnss_leo_msg_combination_node Started.\033[0m"); 
    // ...
    //system("rosbag record -O gnss_leo_data.bag /gnss_leo_msg_combination_node/GNSS_LEO_PsrCarRov1 /gnss_leo_msg_combination_node/GNSS_LEO_Dopp_Array &");
    gnss_leo_msg_combination gnss_leo_msg_combination_(nh);
    ros::spin();
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    // }
    
    return 0;
}
