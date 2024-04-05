#include <iostream>
#include <string.h>
#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <filesystem>
#include <dirent.h>
#include <vector>
#include <algorithm>
#include <sensor_msgs/Imu.h>
#include <velodyne_msgs/VelodyneScan.h>

int main(int argc, char ** argv) {
    printf("Merge ICCV 2023 SLAM Challenge\n");

    std::string data_dir = "/media/dongha/MORIN_POHANG_DATASET/PublicDatasets/iccv_2023/SubT_MRS_Final_Challenge_UGV1/Final_Challenge_UGV1_Rosbag";
    std::vector<std::pair<double, std::string>> bag_time_path_pairs;

    std::string new_bag_path = data_dir + "/merged.bag";

    std::string packet_topic = "/velodyne_packets";
    std::string imu_topic = "/imu/data";

    if(std::filesystem::exists(data_dir.c_str())) {
        DIR * dir_bag = opendir(data_dir.c_str());
        struct dirent * ent;
        if(dir_bag != NULL) {
            while((ent = readdir(dir_bag))!=NULL) {
                std::string file_path = std::string(ent->d_name);
                if(file_path == "." || file_path == "..") {
                    continue;
                }

                size_t begin_loc = file_path.find("raw_data_nuc_") + 12;
                size_t bag_loc = file_path.find(".bag");
                size_t length = bag_loc - begin_loc - 1;

                std::string file_name = file_path.substr(begin_loc + 1, length);

                file_name.erase(std::remove(file_name.begin(), file_name.end(), '-'), file_name.end());
                file_name.erase(std::remove(file_name.begin(), file_name.end(), '_'), file_name.end());
                
                // Convert to integer
                double doubleValue = std::stod(file_name);
                std::pair<double, std::string> pair{doubleValue, file_path};
                bag_time_path_pairs.push_back(pair);
            }
            //sort stuff

            std::sort(bag_time_path_pairs.begin(), bag_time_path_pairs.end(), 
                      [](const auto& lhs, const auto& rhs) {
                return lhs.first < rhs.first;
            });

        }
    } else {
        printf("invalid path\n");
        return -1;
    }

    rosbag::Bag new_bag;
    new_bag.open(new_bag_path, rosbag::bagmode::Write);


    for(int i = 0; i < bag_time_path_pairs.size(); ++i) {
        std::string bag_name = bag_time_path_pairs[i].second;
        std::string bag_path = data_dir + "/" + bag_time_path_pairs[i].second;

        rosbag::Bag data_bag;
        try {
            data_bag.open(bag_path, rosbag::bagmode::Read);
        } catch(const rosbag::BagException & exception) {
            printf("Error opening bag %s\nError: %s", bag_name.c_str(), exception.what());
            continue;
        }

        printf("Opened %s\n", bag_name.c_str());
        rosbag::View view;
        rosbag::View::iterator view_it;

        view.addQuery(data_bag, rosbag::TopicQuery(packet_topic));
        view.addQuery(data_bag, rosbag::TopicQuery(imu_topic));

        double initial_time = view.getBeginTime().toSec();
        double final_time = view.getEndTime().toSec();

        view_it = view.begin();

        while(view_it != view.end()) {
            velodyne_msgs::VelodyneScanConstPtr packet_ptr = view_it->instantiate<velodyne_msgs::VelodyneScan>();
            sensor_msgs::ImuConstPtr imu_ptr = view_it->instantiate<sensor_msgs::Imu>();
            std::string topic_name = view_it->getTopic();
            ros::Time time = view_it->getTime();
            if(topic_name == packet_topic) {
                new_bag.write(packet_topic, time, *packet_ptr);
            }
            if(topic_name == imu_topic) {
                new_bag.write(imu_topic, time, *imu_ptr);
            }

            double percent = (time.toSec() - initial_time)/(final_time-initial_time)*100.0f;
            printf("[%d/%d]%.2f percent done\n",i, static_cast<int>(bag_time_path_pairs.size()) -1, percent);
            ++view_it;
        }

    }
    new_bag.close();




}