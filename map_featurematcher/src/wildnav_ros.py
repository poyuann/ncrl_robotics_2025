#!/usr/bin/python3

"""Core module. Contains the main functions for the project."""
import csv
import cv2
import os
import haversine as hs
from haversine import Unit
import superglue_utils
import numpy as np
import math

from cv_bridge import CvBridge, CvBridgeError
import torch
import rospy
from time import time
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, Imu
from mavros_msgs.msg import State
import threading
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation

current_state = State()

############################################################################################################
# Important variables
############################################################################################################
photo_path =  rospy.get_param('map_dir','')

############################################################################################################
# Class definitios
############################################################################################################
class GeoPhotoDrone:
    """Stores a drone photo together with GNSS location
    and camera rotation parameters
    """
    def __init__(self, lat_origin, lon_origin):
        self.photo = Image()
        self.photo_before = Image()
        self.origin_height = None
        self.origin_width = None
        self.R_w2b = None
        self.latitude = 0
        self.longitude = 0
        self.latitude_calculated = -1
        self.longitude_calculated = -1
        self.altitude = 0
        self.gimball_roll = 0
        self.gimbal_yaw = 0
        self.gimball_pitch = 0
        self.flight_roll = 0
        self.flight_yaw = 0
        self.flight_pitch = 0
        self.corrected = False
        self.matched = 0
        self.last_image_id = None
        self.getImageStatus = False
        self.photo_lock = threading.Lock()
        
        self.color_image = Image()
        self.bridge = CvBridge()

        ##### define pub & sub msg #####
        image_topic = rospy.get_param(
            '~image_topic', '/camera/rgb/image_raw')

        self.pose_msg = PoseStamped()

        #####  publish & subscribe  ######

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback,
                                          queue_size=1, buff_size=52428800)
        self.imu_sub = rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback,
                                          queue_size = 1 , buff_size = 52488000)

        self.pose_pub = rospy.Publisher('/vsnav_pose', PoseStamped, queue_size = 1)
        #####  convert lat lon 2 local  initial ######
        alt_origin = 0
        self.origin_ecef = latlon_to_ecef(lat_origin, lon_origin, alt_origin)
        self.rotation_matrix = build_rotation_matrix(lat_origin, lon_origin)
        self.geo_images_list = read_sat_map(self.origin_ecef, self.rotation_matrix)

    def __str__(self):
        return "%s; \nlatitude: %f \nlongitude: %f \naltitude: %f \ngimball_roll: %f \ngimball_yaw: %f \ngimball_pitch: %f \nflight_roll: %f \nflight_yaw: %f \nflight_pitch: %f" % (self.filename, self.latitude, self.longitude, self.altitude, self.gimball_roll, self.gimball_yaw, self.gimball_pitch, self.flight_roll, self.flight_yaw, self.flight_pitch )
    def image_callback(self, image):
        try:
            with self.photo_lock:
                cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
                self.photo_before = cv_image
                cv_image = self.rotate_image(cv_image, -90)
                self.photo = cv_image
                # cv2.imshow("test", cv_image)
                self.wild_nav()
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            self.photo = None

        cv2.waitKey(3)

    def imu_callback(self, msg):
        imu = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        (self.flight_roll, self.flight_pitch, self.flight_yaw) = euler_from_quaternion(imu)
        
        R_b2w = Rotation.from_quat(imu).as_matrix()
        self.R_w2b = np.linalg.inv(R_b2w)

    def rotate_image(self, image, deg):
        image_center = tuple(np.array(image.shape[1::-1]) / 2)
        rot_mat = cv2.getRotationMatrix2D(image_center, deg, 1.0)
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result
    def wild_nav(self):

        photo = self.photo
        max_features = 0 # keep track of the best match, more features = better match
        located = False # flag to indicate if the drone image was located in the map
        center = None # center of the drone image in the map
        center_px = None

        # Write the query photo to the map folder
        cv2.imwrite(photo_path +"_query_image.png", photo)

        #Call superglue wrapper function to match the query image to the map   
        satellite_map_index_new, center_new, located_image_new, features_mean_new, query_image_new, feature_number = superglue_utils.match_image(photo_path)

        if (feature_number > max_features and center_new[0] < 1 and center_new[1] < 1):
            satellite_map_index = satellite_map_index_new
            center = center_new
            located_image = located_image_new
            features_mean = features_mean_new
            query_image = query_image_new
            max_features = feature_number
            located = True
        photo_name = "Calculated"

        # # If the drone image was located in the map, calculate the geographical location of the drone image
        if center != None and located:        
            loca_map = calculate_geo_pose(self.geo_images_list[satellite_map_index], center, features_mean, query_image.shape )

            # # if you wanna see result of match
            # cv2.imwrite(rospy.get_param('result_dir','')  + photo_name + "_located.png", located_image)


            self.pose_msg.header.stamp = rospy.get_rostime()
            self.pose_msg.pose.position.x = float(loca_map[0])
            self.pose_msg.pose.position.y = float(loca_map[1])
            self.pose_msg.pose.position.z = 40

            self.pose_pub.publish(self.pose_msg)
            # Save the calculated location for later comparison with the ground truth
            self.matched = max_features
            self.last_image_id = satellite_map_index
            self.latitude_calculated = loca_map[0]
            self.longitude_calculated = loca_map[1]
            

        else:
            self.matched = 0
            self.last_image_id = None
            print("NOT MATCHED:", photo_name)

class GeoPhoto:
    """Stores a satellite photo together with (latitude, longitude) for top_left and bottom_right_corner
    """
    def __init__(self, filename, photo, geo_top_left, geo_bottom_right):
        self.filename = filename
        self.photo = photo
        self.top_left_coord = geo_top_left
        self.bottom_right_coord = geo_bottom_right
    def __lt__(self, other):
         return self.filename < other.filename

    def __str__(self):
        return "%s; \n\ttop_left_latitude: %f \n\ttop_left_lon: %f \n\tbottom_right_lat: %f \n\tbottom_right_lon %f " % (self.filename, self.top_left_coord[0], self.top_left_coord[1], self.bottom_right_coord[0], self.bottom_right_coord[1])


############################################################################################################
# Functions for data writing and reading csv files
############################################################################################################

def read_sat_map(origin_ecef, rotation_matrix):
    """Builds a list with satellite geo tagged photos by reading a csv file with this format:
    Filename, Top_left_lat,Top_left_lon,Bottom_right_lat,Bottom_right_long
    "photo_name.png"
    """
    geo_list = []
    my_images = []

    for filename in os.listdir(photo_path):
        temp_name = filename.replace('.png', '')
        txt = temp_name.split('_')
        if (txt[0] == 'tile'):
            img = cv2.imread(os.path.join(photo_path,filename))
            ecef_coords = latlon_to_ecef(float(txt[1]), float(txt[2]), 0)
            current_location = ecef_to_local(ecef_coords, origin_ecef, rotation_matrix)
            geo_photo = GeoPhoto(photo_path + filename,img, (current_location[0]-22, current_location[1]+20), (current_location[0]+22, current_location[1]-20))
            geo_list.append(geo_photo)



    geo_list.sort() # sort alphabetically by filename to ensure that the feature matcher return the right index of the matched sat image
    return geo_list

def calculate_geo_pose(geo_photo, center, features_mean,  shape):
    """
    Calculates the geographical location of the drone image.
    Input: satellite geotagged image, relative pixel center of the drone image, 
    (center with x = 0.5 and y = 0.5 means the located features are in the middle of the sat image)
    pixel coordinatess (horizontal and vertical) of where the features are localted in the sat image, shape of the sat image
    """
    #use ratio here instead of pixels because image is reshaped in superglue    
    latitude = geo_photo.top_left_coord[0] + abs( center[0])  * ( geo_photo.bottom_right_coord[0] - geo_photo.top_left_coord[0])
    longitude = geo_photo.top_left_coord[1] + abs(center[1])  * ( geo_photo.bottom_right_coord[1] - geo_photo.top_left_coord[1])
    return latitude, longitude

def latlon_to_ecef(lat, lon, alt):
    """
    Converts latitude, longitude, and altitude to ECEF coordinates.
    Args:
        lat (float): Latitude in degrees.
        lon (float): Longitude in degrees.
        alt (float): Altitude in meters.
    Returns:
        (x, y, z): ECEF coordinates in meters.
    """
    # WGS84 ellipsoid constants
    a = 6378137.0  # Semi-major axis in meters
    e2 = 6.69437999014e-3  # Square of eccentricity
    
    # Convert latitude and longitude to radians
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)
    
    # Compute prime vertical radius of curvature
    N = a / np.sqrt(1 - e2 * np.sin(lat_rad)**2)
    
    # Compute ECEF coordinates
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e2) + alt) * np.sin(lat_rad)
    
    return x, y, z

def ecef_to_local(ecef_coords, origin_ecef, rotation_matrix):
    """
    Converts ECEF coordinates to a local Gazebo map frame.
    Args:
        ecef_coords (tuple): (x, y, z) in ECEF.
        origin_ecef (tuple): Origin of the local frame in ECEF.
        rotation_matrix (np.ndarray): Rotation matrix from ECEF to local frame.
    Returns:
        (x_local, y_local, z_local): Coordinates in the local frame.
    """
    # Translate to local origin
    translated = np.array(ecef_coords) - np.array(origin_ecef)
    
    # Apply rotation to align with local frame
    local_coords = rotation_matrix @ translated
    
    return local_coords

def build_rotation_matrix(lat_origin, lon_origin):
    """
    Builds the rotation matrix to transform from ECEF to local frame.
    Args:
        lat_origin (float): Latitude of the local origin in degrees.
        lon_origin (float): Longitude of the local origin in degrees.
    Returns:
        np.ndarray: 3x3 rotation matrix.
    """
    lat_rad = np.radians(lat_origin)
    lon_rad = np.radians(lon_origin)
    
    # Define the rotation matrix
    R = np.array([
        [-np.sin(lon_rad),  np.cos(lon_rad), 0],
        [-np.sin(lat_rad)*np.cos(lon_rad), -np.sin(lat_rad)*np.sin(lon_rad), np.cos(lat_rad)],
        [np.cos(lat_rad)*np.cos(lon_rad),  np.cos(lat_rad)*np.sin(lon_rad), np.sin(lat_rad)],
    ])
    
    return R

#######################################
# MAIN
#######################################
def main():

    rospy.init_node("wildnav_ros", anonymous=True)
    geo_photo = GeoPhotoDrone( 24.787961 ,121.001371)


    rospy.spin()

if __name__ == "__main__":

    main()


