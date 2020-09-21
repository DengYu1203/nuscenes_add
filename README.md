# nuscenes_add
###### tags: `nuScenes`
add the nuScenes expanded data

## Download

1. clone the **nuscenes_add** package to your ros workspace
```
git clone https://github.com/DengYu1203/nuscenes_add.git
```
2. download the expand data
3. catkine_make
4. prepare the bags you want to expand the new message
![](https://i.imgur.com/IAy8WaK.png)


## CAN_BUS (imu, pose)
1. download the can_bus data:
![](https://i.imgur.com/8WR9I8L.png)
2. change the bag_fold and imu_fold path to yours
![](https://i.imgur.com/6TuqJ7W.png)
3. roslaunch
```
roslaunch nuscenes_add canbus.launch
```
4. result
![](https://i.imgur.com/HqmpH8a.png)


## lidar segmentation
The intensity of the lidarseg PointCloud is the id of the segmentation which ranges from 0 to 31.
![](https://i.imgur.com/69QYJk3.png)

The pointcloud may look like the picture below:
![](https://i.imgur.com/DmYPTFU.png)


1. download the lidarseg data:
![](https://i.imgur.com/XjUupD4.png)

2. change the bag_fold and lidarseg_fold path to yours
![](https://i.imgur.com/9S1cbKi.png)

3. decide the PointCloud type
true : use the PointXYZRGB(color encoded from id)
false: use the PointXYZI(save the id directly)
![](https://i.imgur.com/mSQpnhw.png)

4. roslaunch
```
roslaunch nuscenes_add lidarseg.launch
```
5. result
![](https://i.imgur.com/ihnDztR.png)

## Bag topics
![](https://i.imgur.com/HxczjFJ.png)
1. can_bus expand topics: **imu** and **pose**
2. lidarseg expand topics: **lidarseg**




