#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/filesystem.hpp>

#include <string>
#include <json/json.h>
#include <json/value.h>
#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl_conversions/pcl_conversions.h"

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/convert.h>

using namespace std;
using namespace boost::filesystem;
using namespace pcl;

bool check_topic = false;

ros::Time fileStamp(string filename){
    long int stamp=stod(filename.c_str());
    ros::Time timestamp;
    timestamp.nsec=(stamp%1000000)*1000;
    timestamp.sec=stamp/1000000;
    return timestamp;
}

sensor_msgs::PointCloud2::Ptr read_bin(path input_file, ros::Time timestamp){
    fstream input(input_file.c_str(), ios::in | ios::binary);
    if(!input.good()){
        std::cout << "Could not read file: " << input_file << endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, ios::beg);
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
    
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr points (new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
    int j;
    for (j=0; input.good() && !input.eof(); j++) {
        velodyne_pointcloud::PointXYZIR point;
        float class_id;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        input.read((char *) &point.ring, sizeof(float));
        points->push_back(point);        
    }
    input.close();

    pcl::toROSMsg(*points, *cloud);
    
    cloud->header.frame_id = "/nuscenes_lidar";
    cloud->header.stamp = timestamp;
    return cloud;
}

void append_bag(string bag_path,string scene,string lidar_ring_path){
    rosbag::Bag bag;
    if(check_topic){
        bag.open(bag_path,rosbag::bagmode::Append | rosbag::bagmode::Read);
        rosbag::View view(bag);
        std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
        std::set<std::string> topics;

        BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
            if(!info->topic.find("lidar_ring")){
                ROS_WARN("This bag has been added the new message, please have a check!");
                return;
            }
        }
    }
    else{
        bag.open(bag_path,rosbag::bagmode::Append);
    }
    string lidar_ring_fold = lidar_ring_path + "/scene-";
    string lidar_ring_name = ".json";
    
    ifstream lidar_ring_in(lidar_ring_fold+scene+lidar_ring_name,ios::binary);
    Json::Value lidar_ring_json;
    Json::Reader reader;
    reader.parse(lidar_ring_in,lidar_ring_json);
    
    cout<<"Read the scene-"+scene+" lidar_ring data";
    cout<<"(including "<<lidar_ring_json.size()-1<<" data)\n";

    for(int index=1;index<lidar_ring_json.size();index++){
        Json::Value temp = lidar_ring_json[index];
        ros::Time timestamp = fileStamp(temp["timestamp"].asString());
        string lidar_bin_path = temp["lidar_original_file"].asCString();
        int start = lidar_bin_path.find("/n");
        int end = lidar_bin_path.find(".bin");
        lidar_bin_path = lidar_bin_path.substr(start,end-start);
        path input_lidar_ring_file = lidar_ring_path+"/lidar_ring_bin"+lidar_bin_path+".bin";
        sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
        cloud = read_bin(input_lidar_ring_file,timestamp);

        bag.write("lidar_ring",timestamp,*cloud);
    }

    bag.close();
}

int main(int argc, char **argv){
    ros::init(argc,argv,"nuscenes_ring");
	ros::NodeHandle nh;
    string bag_fold_s;
    string lidar_ring_fold;
    nh.param<bool>  ("check_topic"    ,check_topic    ,false);
    nh.param<string>("bag_fold"       ,bag_fold_s     ,argv[1]);
    nh.param<string>("lidar_ring_fold"  ,lidar_ring_fold  ,argv[2]);
    path bag_fold = bag_fold_s;
    string os_notation = "/";
    string bag_path;

    cout<<"\033[1;33mReading path:\033[0m\n";
    cout<<"\033[1;33mBag folder: "<<bag_fold.c_str()<<"\033[0m"<<endl;
    cout<<"\033[1;33mLiDAR ring folder: "<<lidar_ring_fold<<"\033[0m\n"<<endl;
    for (auto i = directory_iterator(bag_fold); i != directory_iterator(); i++)
    {
        string in;
        if (!is_directory(i->path())) // read files in fold
        {
			in = i->path().filename().string();
            int pos = in.find("scene-");
            int end = in.find(".");
            string scene = in.substr(pos+6,end-pos-6);
			bag_path = bag_fold.c_str() + os_notation + in;
			cout<<"---------------------------------------------------------------\n";
            cout<<"Read bag: "<<bag_path<<endl;
            append_bag(bag_path,scene,lidar_ring_fold);
        }
        else
            continue;
    }
    
}