#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <boost/filesystem.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <json/json.h>
#include <json/value.h>
#include <vector>
#include <cmath>

#include <nuscenes_pose/pose.h>

using namespace tf;
using namespace std;
using namespace boost::filesystem;

ros::Time fileStamp(string filename){
    
    long int stamp=stod(filename.c_str());
    ros::Time timestamp;
    timestamp.nsec=(stamp%1000000)*1000;
    timestamp.sec=stamp/1000000;
    return timestamp;
}
void append_bag(string bag_path,string imu_scene,string imu_path){
    rosbag::Bag bag;
    bag.open(bag_path,rosbag::bagmode::Append);
    
    string imu_fold = imu_path + "/scene-";
    string imu_name = "_ms_imu.json";
    
    ifstream imu_in(imu_fold+imu_scene+imu_name,ios::binary);
    Json::Value imu_json;
    Json::Reader reader;
    reader.parse(imu_in,imu_json);
    
    cout<<"Read the scene-"+imu_scene+" imu data";
    cout<<"(including "<<imu_json.size()<<" imu data)\n";
    sensor_msgs::Imu imu;

    for(int index=0;index<imu_json.size();index++){
        Json::Value temp = imu_json[index];
        // cout<<index<<endl;
        imu.header.frame_id = "/car";
        imu.header.seq = index;
        imu.header.stamp = fileStamp(temp["utime"].asString());   // time stamp
        
        imu.angular_velocity.x = temp["rotation_rate"][0].asDouble();
        imu.angular_velocity.y = temp["rotation_rate"][1].asDouble();
        imu.angular_velocity.z = temp["rotation_rate"][2].asDouble();
        
        imu.linear_acceleration.x = temp["linear_accel"][0].asDouble();
        imu.linear_acceleration.y = temp["linear_accel"][1].asDouble();
        imu.linear_acceleration.z = temp["linear_accel"][2].asDouble();
        
        imu.orientation.w = temp["q"][0].asDouble();
        imu.orientation.x = temp["q"][1].asDouble();
        imu.orientation.y = temp["q"][2].asDouble();
        imu.orientation.z = temp["q"][3].asDouble();

        bag.write("imu",imu.header.stamp,imu);
    }
    
    
    string pose_name = "_pose.json";
    ifstream pose_in(imu_fold+imu_scene+pose_name,ios::binary);
    Json::Value pose_json;
    reader.parse(pose_in,pose_json);
    cout<<"Read the scene-"+imu_scene+" pose data";
    cout<<"(including "<<pose_json.size()<<" pose data)\n";

    for (int index=0;index<pose_json.size();index++){
        nuscenes_pose::pose pose;
        Json::Value temp = pose_json[index];

        pose.header.frame_id = "/map";
        pose.header.seq = index;
        pose.header.stamp = fileStamp(temp["utime"].asString());   // time stamp
        
        pose.position.x = temp["pos"][0].asDouble();
        pose.position.y = temp["pos"][1].asDouble();
        pose.position.z = temp["pos"][2].asDouble();

        pose.acceleration.x = temp["accel"][0].asDouble();
        pose.acceleration.y = temp["accel"][1].asDouble();
        pose.acceleration.z = temp["accel"][2].asDouble();
        
        pose.angular_velocity.x = temp["rotation_rate"][0].asDouble();
        pose.angular_velocity.y = temp["rotation_rate"][1].asDouble();
        pose.angular_velocity.z = temp["rotation_rate"][2].asDouble();

        pose.velocity.x = temp["vel"][0].asDouble();
        pose.velocity.y = temp["vel"][1].asDouble();
        pose.velocity.z = temp["vel"][2].asDouble();
        
        pose.orientation.w = temp["orientation"][0].asDouble();
        pose.orientation.x = temp["orientation"][1].asDouble();
        pose.orientation.y = temp["orientation"][2].asDouble();
        pose.orientation.z = temp["orientation"][3].asDouble();

        bag.write("pose",pose.header.stamp,pose);        
    }

    bag.close();
}

int main(int argc, char **argv){
    ros::init(argc,argv,"nuscenes_add");
	ros::NodeHandle nh;
    // ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu",10);
    
    path bag_fold = argv[1];
    string os_notation = "/";
    string bag_path;

    string imu_fold = argv[2];
    cout<<"\033[1;33mReading path:\033[0m\n";
    cout<<"\033[1;33mBag fold: "<<bag_fold.c_str()<<"\033[0m"<<endl;
    cout<<"\033[1;33mImu and Pose fold: "<<imu_fold<<"\033[0m\n"<<endl;
    for (auto i = directory_iterator(bag_fold); i != directory_iterator(); i++)
    {
        string in;
        if (!is_directory(i->path())) // read files in fold
        {
			in = i->path().filename().string();
            int pos = in.find("scene-");
            int end = in.find(".");
            string imu_scene = in.substr(pos+6,end-pos-6);
			bag_path = bag_fold.c_str() + os_notation + in;
			cout<<"---------------------------------------------------------------\n";
            cout<<"Read bag: "<<bag_path<<endl;
            append_bag(bag_path,imu_scene,imu_fold);
        }
        else
            continue;
    }
    
}