## ika Dataset for Lidar Object Detection

### :arrow_forward: [Download](https://rwth-aachen.sciebo.de/s/85Dd9YVqEupu8Fr)

### Folder Structure

Lidar point clouds are stored in `pointcloud`, corresponding object list labels are stored in `object_list`.

```
ika-dataset
├── class_ids.json
├── pointcloud
│   ├── 1565340816911679.pcd
│   └── ...
└── object_list
    ├── 1565340816911679.csv
    └── ...
```

### Object List Format

The object list labels are stored in CSV files containing one row per object. The column description can be found in the header (first row) of each file.

```
id, existence_prob, class, class_prob, x, y, z, vel_abs, acc_abs, yaw, yaw_rate, width, length, height
```

The interesting quantities for the object detection algorithm are the object's pose (`x`, `y`, `z`, `yaw`), its size (`width`, `length`, `height`) and its semantic class (`class`). Note that `class` is an integer ID mapping to one of the classes defined in `class_ids.json`.
