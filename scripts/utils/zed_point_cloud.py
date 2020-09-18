import rospy
from datetime import datetime
# from sensor_msgs.msg import LaserScan, PointCloud2, Image
# from std_msgs import msg
import os, cv2 as cv, pdb, message_filters, numpy as np, time
import pyzed.sl as sl


class ZedUtils(object):
    def __init__(self):
        return

    def record():
        # Create a ZEDCamera object
        zed = sl.Camera()

        # Create a InitParameters object and set configuration parameters
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 video mode (default fps: 60)
        # Use a right-handed Y-up coordinate system
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
        init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

        # Open the camera
        err = zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Enable positional tracking with default parameters.
        # Positional tracking needs to be enabled before using spatial mapping
        py_transform = sl.Transform()
        tracking_parameters = sl.PositionalTrackingParameters(init_pos=py_transform)
        err = zed.enable_positional_tracking(tracking_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Enable spatial mapping
        mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD)
        err = zed.enable_spatial_mapping(mapping_parameters)
        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)

        # Grab data during 3000 frames
        i = 0
        py_fpc = sl.FusedPointCloud()  # Create a Mesh object
        runtime_parameters = sl.RuntimeParameters()

        while i < 3000:
            # For each new grab, mesh data is updated
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                # In the background, spatial mapping will use newly retrieved images, depth and pose to update the mesh
                mapping_state = zed.get_spatial_mapping_state()

                print("\rImages captured: {0} / 3000 || {1}".format(i, mapping_state))

                i = i + 1

        print("\n")

        # Extract, filter and save the mesh in an obj file
        print("Extracting Point Cloud...\n")
        err = zed.extract_whole_spatial_map(py_fpc)
        print(repr(err))
        #print("Filtering Mesh...\n")
        #py_mesh.filter(sl.MeshFilterParameters())  # Filter the mesh (remove unnecessary vertices and faces)
        print("Saving Point Cloud...\n")
        py_fpc.save("/media/nvidia/data/rosbag/zed/zed.obj")

        # Disable tracking and mapping and close the camera
        zed.disable_spatial_mapping()
        zed.disable_positional_tracking()
        zed.close()