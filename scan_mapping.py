import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2


def get_depth_scale(pipe):
    #this method procures the scale constant to convert depth from raw images to meters
    profile = pipe.get_active_profile()
    dev = profile.get_device() #right now, D435i is the only device so this works fine
    scale = dev.first_depth_sensor().get_depth_scale()
    return scale


#return all six axes of the pose from T265
def get_pose(frames):
    #return pose and translation as a 6-long numpy array
    pose = frames.get_pose_frame()
    if !(pose):
        return np.asarray([], np.float32)
    #Note: Rotation in returned as a quaternion
    w = data.rotation.w
    x = -data.rotation
    

def scan_points(image_frame, intren, depth_scale):
    scan_row = int(image_frame.shape[0] / 2)
    row_size = image_frame.shape[1]

    scan_mat = np.zeros((row_size, row_size), np.uint8)
    MAX_WIDTH = 2 #screen covers a maximum of 10 meters of width
    scale_val = (row_size - 10) / (MAX_WIDTH) #scale points to make them fit on image
    
    point_collection = []
    for j in range(row_size):
        depth_val = image_frame[scan_row][j]
        point = rs.rs2_deproject_pixel_to_point(intren, [j, scan_row], depth_val)
        point = np.asarray(point) * depth_scale
        point_collection.append(point)

    return np.asarray(point_collection, np.float32)


def frame_to_scan(image_frame, intren, depth_scale=0.5):
    scan_row = int(image_frame.shape[0] / 2)
    row_size = image_frame.shape[1]

    scan_mat = np.zeros((row_size, row_size), np.uint8)
    MAX_WIDTH = 2 #screen covers a maximum of 10 meters of width
    scale_val = (row_size - 10) / (MAX_WIDTH) #scale points to make them fit on image
    
    point_collection = []
    for j in range(row_size):
        depth_val = image_frame[scan_row][j]
        point = rs.rs2_deproject_pixel_to_point(intren, [j, scan_row], depth_val)
        point = np.asarray(point) * depth_scale
        point_collection.append(point)

        print("x_coordinate: ", point[0])

        img_point = scale_val * point
        img_x_val = int(img_point[0]) + int(row_size / 2) - 1

        print("img_x, img_y: " + str(img_point[0]) + ", " + str(img_point[2]))
        #Dont draw any points that dont fall on the image plane
        if (img_x_val >= 0 and img_x_val < row_size) and (img_point[2] >= 0 and img_point[2] <= row_size):
            print("writing!")
            scan_mat[row_size - int(img_point[2]) - 1][img_x_val] = 255

    return scan_mat    


def map_environment(frames):
    scan_points = 
    pass


def map_func():
    pipe = rs.pipeline()
    pipe.start()

    hole_fill = rs.hole_filling_filter()
    DEPTH_SCALE = get_depth_scale(pipe)

    map_point_cloud = np.asarray([], np.float32)
    while True:
        map_point_cloud = map_environment(map_point_cloud)



def main():
    print("Accumulative Scanning Map")
    print("Still under construction...")

    map_func()
    

if __name__ == "__main__"
    main()