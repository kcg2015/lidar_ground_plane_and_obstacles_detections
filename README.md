# lidar_ground_plane_and_obstacles_detections
This repo include Python and C++ implementations/examples that show how to process 3-D Lidar data by segmenting the ground plane and finding obstacles.



![img](Python/figs/result_XY.gif)

For the Python implementation, I use strawlab/python-pcl (https://github.com/strawlab/python-pcl), a small python binding to the point cloud library (PCL). As of August 2019, the majority of the wrapped APIs only work with `PointXYZ` point type. If the point type is of the format of `PointXYZI`, simple conversion is required, as shown in the following example.

```
cloud_XYZI = pcl.load_XYZI('data_1/0000000006.pcd')
cloud_np = np.array(cloud_XYZI)
cloud_XYZ = pcl.PointCloud()
cloud_XYZ.from_array(cloud_np[:, 0:3])
```

### Down-sampling
#### Voxel Grid
Voxel grid filtering will create a cubic grid and will filter the cloud by only leaving a single point per voxel cube, so the larger the cube length the lower the resolution of the point cloud.

The key parameters of voxel grid filtering is the leaf size.

```
def voxel_filter(cloud, leaf_size):
    """
    Input parameters:
    cloud: input point cloud to be filtered
    leaf_size: a list of leaf_size for X, Y, Z 
    Output:
    cloud_voxel_filtered: voxel-filtered cloud
    """
    sor = cloud.make_voxel_grid_filter()
    size_x, size_y, size_z = leaf_size
    sor.set_leaf_size(size_x, size_y, size_z)
    cloud_voxel_filtered = sor.filter()
    
    return cloud_voxel_filtered

```

Here, a `cloud.make_voxel_grid_filter()` filter is created with a leaf size specified by `leaf_size', the input data is passed, and the output is computed and stored in cloud_filtered.

#### Region of Interest (ROI)

First a 3D box is specified. Any points outside that box are filtered out. 

```
def roi_filter(cloud, x_roi, y_roi, z_roi):
    """
    Input Parameters:
        cloud: input point cloud
        x_roi: ROI range in X
        y_roi: ROI range in Y
        z_roi: ROI range in Z
    
    Output:    
        ROI region filtered point cloud
    """
    clipper = cloud.make_cropbox()
    cloud_roi_filtered= pcl.PointCloud()
    xc_min, xc_max = x_roi
    yc_min, yc_max = y_roi
    zc_min, zc_max = z_roi
    clipper.set_MinMax(xc_min, yc_min, zc_min, 0, xc_max, yc_max, zc_max, 0)
    cloud_roi_filtered =clipper.filter()
    return cloud_roi_filtered
```
