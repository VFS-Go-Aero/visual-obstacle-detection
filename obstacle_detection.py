import pyzed.sl as sl
import numpy as np
import open3d as o3d
import sys

def main():
    # Define safety zone parameters (in meters)
    safety = 8  # Maximum distance in each axis to consider
    safety_sq = 64  # Square of safety distance for radial check (8^2 = 64)

    # Initialize ZED camera
    zed = sl.Camera()

    # Configure camera initialization parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720 # Use HD720 video mode (default fps: 60)
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Use a right-handed Y-up coordinate system
    init_params.coordinate_units = sl.UNIT.METER # Set units in meters

    # Attempt to open the camera
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        sys.exit(1)

    # Configure positional tracking parameters
    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.set_as_static = False  # Camera is moving, not static

    # Enable positional tracking for spatial mapping
    if zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to enable tracking")
        zed.close()
        sys.exit(1)

    # Configure spatial mapping parameters
    mapping_parameters = sl.SpatialMappingParameters()
    mapping_parameters.resolution_meter = 0.03  # Resolution of 3cm per voxel
    mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD  # Use fused point cloud format
    mapping_parameters.range_meter = 5.0  # Maximum mapping range of 5 meters
    mapping_parameters.max_memory_usage = 1024  # Limit memory usage to 1GB
    mapping_parameters.use_chunk_only = True  # Use chunked spatial mapping

    # Enable spatial mapping with configured parameters
    if zed.enable_spatial_mapping(mapping_parameters) != sl.ERROR_CODE.SUCCESS:
        print("Failed to enable spatial mapping")
        zed.close()
        sys.exit(1)

    # Initialize point cloud container and frame counter
    point_cloud = sl.FusedPointCloud()
    timer = 0

    # Main processing loop
    while 1:
        # Grab a new frame from the camera
        if zed.grab() == sl.ERROR_CODE.SUCCESS:
            # Stop after 10 frames (for testing/demo purposes)
            if timer == 10:
                break
    
            # Request an update of the spatial map every 30 frames (0.5s in HD720 mode)
            if timer % 30 == 0:
                zed.request_spatial_map_async()
    
            # Retrieve spatial_map when ready.
            # Note: timer > 0 intentionally skips retrieval on frame 0. The first request is sent at
            # timer == 0, and retrieval is deferred to subsequent frames once the async request is ready.
            if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS and timer > 0:
                # Get the updated point cloud
                zed.retrieve_spatial_map_async(point_cloud)
                points = point_cloud.vertices()
                
                # Skip processing if no points were captured
                if points.size == 0:
                    continue
                
                # Extract XYZ coordinates
                xyz = points[:, :3]
                # Remove invalid points (NaN or Inf values)
                xyz = xyz[np.isfinite(xyz).all(axis=1)]
                # Ensure contiguous array for better performance
                xyz = np.ascontiguousarray(xyz, dtype=np.float32)

                # Original loop-based filtering approach (commented out for reference)
                #selected_points = []
                #for point in xyz:
                #    x = point[0]
                #    y = point[1]
                #    z = point[2]
                #    if (x >= safety or y >= safety or z >= safety):
                #        continue;
                #    if ((x**2 + y**2 + z**2) > safety_sq):
                #        continue;
                #    selected_points.append([x, y, z])
                
                # Filter points within safety zone using vectorized operations
                # Keep points where x, y, z < safety AND distance from origin < safety
                mask = (
                    (xyz[:, 0] < safety) &
                    (xyz[:, 1] < safety) &
                    (xyz[:, 2] < safety) &
                    ((xyz ** 2).sum(axis=1) < safety_sq)
                )
                selected_points = xyz[mask]

                # If no points remain after filtering, skip clustering for this frame
                if selected_points.size == 0:
                    print("No points within safety zone; skipping clustering for this frame.")
                else:
                    # Create Open3D point cloud for clustering
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(selected_points)

                    # Perform DBSCAN clustering to identify distinct objects
                    # eps=0.2: maximum distance between points in a cluster (20cm)
                    # min_points=10: minimum points required to form a cluster
                    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
                        labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10))

                    # Calculate number of detected objects, excluding noise label -1
                    unique_labels = np.unique(labels)
                    valid_labels = unique_labels[unique_labels >= 0]

                    if valid_labels.size == 0:
                        print("No clusters detected (all points are noise).")
                    else:
                        num_objects = len(valid_labels)
                        print("Detected " + str(num_objects) + " objects")

                        # Calculate and print centroid for each detected object
                        for idx, label in enumerate(valid_labels):
                            cluster_points = np.asarray(pcd.points)[labels == label]
                            centroid = cluster_points.mean(axis=0)
                            print("Object " + str(idx) + ": centroid " + str(centroid))
            # Increment frame counter
            timer += 1

    # Cleanup: disable features and close camera
    zed.disable_spatial_mapping()
    zed.disable_tracking()
    zed.close()


if __name__ == "__main__":
    main()
