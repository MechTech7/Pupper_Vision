import numpy as np
import pyrealsense2 as rs
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
    pose_frame = frames.get_pose_frame()
    if not pose_frame:
        print("no pose found!")
        return np.asarray([], np.float32)
    pose = pose_frame.get_pose_data()
    #Note: Rotation in returned as a quaternion
    w = pose.rotation.w
    x = -pose.rotation.x
    y = pose.rotation.x
    z = -pose.rotation.y
    return np.asarray([w, x, y, z], np.float32)

    

def scan_points(image_frame, intren, depth_scale):
    scan_row = int(image_frame.shape[0] / 2)
    row_size = image_frame.shape[1]

    MAX_WIDTH = 2 #screen covers a maximum of 10 meters of width
    
    point_collection = []
    for j in range(row_size):
        depth_val = image_frame[scan_row][j]
        point = rs.rs2_deproject_pixel_to_point(intren, [j, scan_row], depth_val)
        point = np.asarray(point) * depth_scale
        point_collection.append(point)

    return np.asarray(point_collection, np.float32)





def map_environment(frames, map_point_cloud):
    pose = get_pose(frames)

    return np.asarray([], np.float32)

def get_pipes():
    context = rs.context()

    pipe_list = []
    depth_scale = 0.0
    for dev in context.query_devices():
        print("device: ", dev)
        
        cfg = rs.config()
        
        cam_inf = rs.camera_info
        serial = dev.get_info(cam_inf.serial_number)
        cfg.enable_device(serial)

        pipe = rs.pipeline(context)
        pipe.start(cfg)
        pipe_list.append(pipe)

        #get scaling value from depth camera
        
        dev_sensors = dev.query_sensors()
        for sens in dev_sensors:
            if depth_scale == 0.0 and sens.is_depth_sensor():
                depth_scale = sens.as_depth_sensor().get_depth_scale()
                print("depth scale found! ", depth_scale)
                break
    
    return pipe_list, depth_scale

    



def map_func():
    pipe_list, DEPTH_SCALE = get_pipes()
    
    print("{} pipes created!".format(len(pipe_list)))
    map_point_cloud = np.asarray([], np.float32)
    while True:
        depth_frame = None
        pose_frame = None
        count = 0
        for pipe in pipe_list:
            print("trying pipe: ", count)
            count += 1
            frames = pipe.wait_for_frames()
            dp_frame = frames.get_depth_frame()
            if dp_frame:
                #if its a depth frame...
                print("depth frame recieved")
                depth_frame = dp_frame
            
            print("not a depth frame")
            ps_frame = frames.get_pose_frame()
            if ps_frame:
                print("pose frame recieved") 
                pose_frame = ps_frame

            else:
                print("Note: neither depth or pose frames available on this pipe (what's it even for?)")

            print("-----------------")
        #map_point_cloud = map_environment(frames, map_point_cloud)



def main():
    print("Accumulative Scanning Map")
    print("Still under construction...")

    map_func()
    

if __name__ == "__main__":
    main()