import pyzed.sl as sl
import numpy as np
import open3d as o3d
import sys

def main():
    safety = 8
    safety_sq = 64

    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720 # Use HD720 video mode (default fps: 60)
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Use a right-handed Y-up coordinate system
    init_params.coordinate_units = sl.UNIT.METER # Set units in meters

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera")
        sys.exit(1)

    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.set_as_static = False

    if zed.enable_positional_tracking(tracking_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to enable tracking")
        zed.close()
        sys.exit(1)

    mapping_parameters = sl.SpatialMappingParameters()
    mapping_parameters.resolution_meter = 0.03
    mapping_parameters.map_type = sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
    mapping_parameters.range_meter = 5.0
    mapping_parameters.max_memory_usage = 1024
    mapping_parameters.use_chunk_only = True

    if zed.enable_spatial_mapping(mapping_parameters) != sl.ERROR_CODE.SUCCESS:
        print("Failed to enable spatial mapping")
        zed.close()
        sys.exit(1)


    point_cloud = sl.FusedPointCloud()
    timer = 0

    while 1:
      if zed.grab() == sl.ERROR_CODE.SUCCESS:
          if timer == 10:
              break
    
          # Request an update of the spatial map every 30 frames (0.5s in HD720 mode)
          if timer % 30 == 0 :
             zed.request_spatial_map_async()
    
          # Retrieve spatial_map when ready
          if zed.get_spatial_map_request_status_async() == sl.ERROR_CODE.SUCCESS and timer > 0:
             zed.retrieve_spatial_map_async(point_cloud)
             points = point_cloud.vertices()
             if points.size == 0:
                 continue
             xyz = points[:, :3]
             xyz = xyz[np.isfinite(xyz).all(axis=1)]
             xyz = np.ascontiguousarray(xyz, dtype=np.float32)

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

            selected_points = np.array([point for point in xyz 
                            if x < safety and y < safety and z < safety 
                            and (point[0]**2 + point[1]**2 + point[2]**2) < safety_sq],
                           dtype=np.float32)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(selected_points)

            with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error) as cm:
                labels = np.array(pcd.cluster_dbscan(eps=0.2, min_points=10))

            num_objects = labels.max() + 1
            print("Detected " + num_objects + " objects")

            for i in range(num_objects):
                cluster_points = np.asarray(pcd.points)[labels == i]
                centroid = cluster_points.mean(axis=0)
                print("Object " + i + ": centroid " + centroid)


          timer += 1

    zed.disable_spatial_mapping()
    zed.disable_tracking()
    zed.close()


if __name__ == "__main__":
    main()
