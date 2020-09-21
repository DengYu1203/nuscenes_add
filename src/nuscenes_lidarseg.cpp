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

using namespace std;
using namespace boost::filesystem;
using namespace pcl;

bool check_topic = false;
bool use_rgb = true;   // to decide publish the PointXYZRGB(true) or PointXYZI(false)

// typedef std::tuple<int,int,int> rgb;
typedef struct color
{
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
}rgb;

enum class class_id{
    noise,
    human_pedestrian_adult,
    human_pedestrian_child,
    human_pedestrian_wheelchair,
    human_pedestrian_stroller,
    human_pedestrian_personal_mobility,
    human_pedestrian_police_officer,
    human_pedestrian_construction_worker,
    animal,
    vehicle_car,
    vehicle_motorcycle,
    vehicle_bicycle,
    vehicle_bus_bendy,
    vehicle_bus_rigid,
    vehicle_truck,
    vehicle_construction,
    vehicle_emergency_ambulance,
    vehicle_emergency_police,
    vehicle_trailer,
    movable_object_barrier,
    movable_object_trafficcone,
    movable_object_pushable_pullable,
    movable_object_debris,
    static_object_bicycle_rack,
    flat_driveable_surface,
    flat_sidewalk,
    flat_terrain,
    flat_other,
    static_manmade,
    static_vegetation,
    static_other,
    vehicle_ego
};

rgb annotation[32] = {  {0,0,0},
                        {83,130,176},
                        {0,32,221},
                        {150,205,232},
                        {109,150,230},
                        {206,119,146},
                        {0,13,123},
                        {226,134,131},
                        {125,60,218},
                        {115,128,142},
                        {198,110,51},
                        {105,105,105},
                        {54,78,79},
                        {181,145,144},
                        {203,49,66},
                        {240,133,91},
                        {236,84,40},
                        {242,162,58},
                        {222,153,85},
                        {237,95,42},
                        {250,214,73},
                        {236,79,102},
                        {241,145,53},
                        {238,108,82},
                        {92,204,191},
                        {161,33,76},
                        {68,12,72},
                        {128,177,78},
                        {217,184,141},
                        {251,228,200},
                        {78,171,49},
                        {252,241,245}};

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
    if(use_rgb){
        pcl::PointCloud<PointXYZRGB>::Ptr points (new pcl::PointCloud<PointXYZRGB>);
        int j;
        for (j=0; input.good() && !input.eof(); j++) {
            PointXYZRGB point;
            float class_id;
            input.read((char *) &point.x, 3*sizeof(float));
            input.read((char *) &class_id, sizeof(float));
            point.r = annotation[int(class_id)].r;
            point.g = annotation[int(class_id)].g;
            point.b = annotation[int(class_id)].b;
            points->push_back(point);        
        }
        input.close();

        pcl::toROSMsg(*points, *cloud);
    }
    else{
        pcl::PointCloud<PointXYZI>::Ptr points (new pcl::PointCloud<PointXYZI>);
        int j;
        for (j=0; input.good() && !input.eof(); j++) {
            PointXYZI point;
            float class_id;
            input.read((char *) &point.x, 3*sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            points->push_back(point);        
        }
        input.close();

        pcl::toROSMsg(*points, *cloud);
    }
    cloud->header.frame_id = "/nuscenes_lidar";
    cloud->header.stamp = timestamp;
    return cloud;
}

void append_bag(string bag_path,string scene,string lidarseg_path){
    rosbag::Bag bag;
    if(check_topic){
        bag.open(bag_path,rosbag::bagmode::Append | rosbag::bagmode::Read);
        rosbag::View view(bag);
        std::vector<const rosbag::ConnectionInfo *> connection_infos = view.getConnections();
        std::set<std::string> topics;

        BOOST_FOREACH(const rosbag::ConnectionInfo *info, connection_infos) {
            if(!info->topic.find("lidarseg")){
                ROS_WARN("This bag has been added the new message, please have a check!");
                return;
            }
        }
    }
    else{
        bag.open(bag_path,rosbag::bagmode::Append);
    }
    string lidarseg_fold = lidarseg_path + "/scene-";
    string lidarseg_name = ".json";
    
    ifstream lidarseg_in(lidarseg_fold+scene+lidarseg_name,ios::binary);
    Json::Value lidarseg_json;
    Json::Reader reader;
    reader.parse(lidarseg_in,lidarseg_json);
    
    cout<<"Read the scene-"+scene+" lidarseg data";
    cout<<"(including "<<lidarseg_json.size()-1<<" data)\n";

    for(int index=1;index<lidarseg_json.size();index++){
        Json::Value temp = lidarseg_json[index];
        ros::Time timestamp = fileStamp(temp["timestamp"].asString());
        
        path input_lidarseg_file = lidarseg_path+"/lidarseg_bin/"+temp["lidar_seg_file"].asCString();
        sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2);
        cloud = read_bin(input_lidarseg_file,timestamp);

        bag.write("lidarseg",timestamp,*cloud);
    }

    bag.close();
}

int main(int argc, char **argv){
    ros::init(argc,argv,"nuscenes_lidarseg");
	ros::NodeHandle nh;
    string bag_fold_s;
    string lidarseg_fold;
    nh.param<bool>  ("check_topic"    ,check_topic    ,false);
    nh.param<bool>  ("use_rgb"        ,use_rgb        ,true);
    nh.param<string>("bag_fold"       ,bag_fold_s     ,argv[1]);
    nh.param<string>("lidarseg_fold"  ,lidarseg_fold  ,argv[2]);
    path bag_fold = bag_fold_s;
    string os_notation = "/";
    string bag_path;

    cout<<"\033[1;33mReading path:\033[0m\n";
    cout<<"\033[1;33mBag folder: "<<bag_fold.c_str()<<"\033[0m"<<endl;
    cout<<"\033[1;33mLiDAR seg folder: "<<lidarseg_fold<<"\033[0m\n"<<endl;
    cout<<"\033[1;33mPointCloud type: "<<(use_rgb ? "PointXYZRGB":"PointXYZI")<<"\033[0m\n"<<endl;
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
            append_bag(bag_path,scene,lidarseg_fold);
        }
        else
            continue;
    }
    
}