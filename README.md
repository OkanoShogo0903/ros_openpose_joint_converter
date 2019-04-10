# What is this???
ros-openpose publish joint coordinate.  
But it is not suitable for some machine learning model.  
So, **this package convert ros-openpose topic to angle topic**.  

[qiita](https://qiita.com/okanosyogo/items/d62e6471ca3bb2877b0c<Paste>)

## requirement
```
openpose(**coco weight!!**)
ros-openpose
```

## How to use
```
$ chmod +x scripts/main.py
$ roslaunch ros_openpose_joint_converter all.launch 
```
**In default, missing value is `0`.**
**If you want to change missing value, please eddit launch file.**
You will be change it so eazy;)

### Broadcast
- Topic
`/joint_angle`
```
std_msgs/Int32MultiArray[] persons
uint32 num
```

- Example

Two person.  
Zero mean missing value.
```
persons: 
  - 
    layout: 
      dim: []
      data_offset: 0
    data: [22, 0, 0, 56, 76, 0, 25, 29, 1, 0, 0, 0, 0, 34, 0, 0]
  - 
    layout: 
      dim: []
      data_offset: 0
    data: [40, 49, 4, 41, 52, 3, 10, 37, 10, 41, 8, 0, 0, 8, 0, 0]
num: 2
```
