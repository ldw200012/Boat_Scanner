# PillarReID
PillarReID using raw/completed/reconstructed data

## 0. How to Run
### 0.1. Filter Boat Point Cloud from Raw Rosbag
```
rosbag play {rosbag_with_cam_and_lidar.bag}
```
```
roslaunch vessel_pointcloud_scanner vessel_pointcloud_scanner.launch
```

### 0.2. Save Boat Point Cloud into Dataset Folder
Change the Line 113 "pillar_extraction_node = PointPillarNode(CLASS_IDX)",
where CLASS_IDX should be string value of class index (e.g. "0008")
```
cd PillarReID/pillars_feature_extractor/scripts
python create_dataset.py
```
