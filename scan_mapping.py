import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import math as m
import cv2

MAX_POINT_COUNT = 10e7 #maximum number of points that can be held in the point map
def get_depth_scale(pipe):
    #this method procures the scale constant to convert depth from raw images to meters
    profile = pipe.get_active_profile()
    dev = profile.get_device() #right now, D435i is the only device so this works fine
    scale = dev.first_depth_sensor().get_depth_scale()
    return scale


#return all six axes of the pose from T265
def get_pose(pose_frame):
    #return pose and translation as a 6-long numpy array
    if not pose_frame:
        print("no pose found!")
        return np.asarray([], np.float32), np.asarray([], np.float32)
    data = pose_frame.get_pose_data()
    #Note: Rotation in returned as a quaternion
    #Euler angles from quaternion

    w = data.rotation.w
    x = -data.rotation.z
    y = data.rotation.x
    z = -data.rotation.y

    pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi
    roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi
    yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi

    quat_arr = np.asarray([-data.rotation.x, -data.rotation.y, -data.rotation.z, data.rotation.w], np.float32)
    #np.asarray([pitch, yaw, roll], np.float32)

    return quat_arr, np.asarray([data.translation.x, data.translation.y, data.translation.z], np.float32)

def scan_portrait(points, current_pos, min_height = -0.2, max_height = 0.2, panel_width = 1000):
    #min height and max height are minimum and maximum heights that will be included in the scan portrait
    #output a numpy matrix that is effectively a slice of the points that are provided

    MAX_WIDTH = 8

    #Note: this display is O(N) so its probably really slow.  
    portrait_mat = np.zeros([panel_width, panel_width], np.uint8)  
    size = points.shape[0]

    scale_val = (panel_width - 10) / (MAX_WIDTH)

    center = int(panel_width / 2)

    write_count = 0
    for i in range(size):
        curr_point = points[i]
        if (curr_point[1] > max_height) or (curr_point[1] < min_height):
            continue

        #distance caluclation
        diff_vec = curr_point + current_pos
        x_dist = diff_vec[0]
        y_dist = diff_vec[2]

        x_val = (x_dist * scale_val) + center - 1
        y_val = (y_dist * scale_val) + center - 1

        if (x_val >= 0 and x_val < panel_width) and (y_val >= 0 and y_val <= panel_width):
            write_count += 1 
            portrait_mat[panel_width - int(y_val) - 1][int(x_val)] = 255

    print("Wrote {} point".format(write_count))
    return portrait_mat




        


def scan_points(image_frame, intren, depth_scale):
    scan_row = int(image_frame.shape[0] / 2)
    row_size = image_frame.shape[1]
    
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
        

        type_val = 'p' #An additional value that allows the cameras to be differentiated

        #get scaling value from depth camera
        dev_sensors = dev.query_sensors()
        for sens in dev_sensors:
            if depth_scale == 0.0 and sens.is_depth_sensor():
                type_val = 'd'
                depth_scale = sens.as_depth_sensor().get_depth_scale()
                print("depth scale found! ", depth_scale)
                break
        
        pipe_list.append((pipe, type_val))
            
        
    return pipe_list, depth_scale

    
def get_frames(pipe_list, filter=None):
    depth_frame = None
    pose_frame = None

    count = 0
    for pipe_pair in pipe_list:
        pipe = pipe_pair[0]
        type_val = pipe_pair[1]

        print("trying pipe: ", count)

        count += 1
        frames = pipe.wait_for_frames()

        
        if type_val == 'd':
            #if its a depth frame...
            print("depth frame recieved")
            depth_frame = frames.get_depth_frame()

            if filter != None:
                print ("decimating!")
                depth_frame = filter.process(depth_frame)
            continue


        if type_val == 'p':
            #if its a pose frame
            print("pose frame recieved") 
            pose_frame = frames.get_pose_frame()

        else:
            print("Note: neither depth or pose frames available on this pipe (what's it even for?)")

    return [depth_frame, pose_frame]

def translate_frame_to_points(frameset, depth_scale):
    #returns the points rotated and translated
    #Note: add the translation between the depth camera and the translation center for the current holder
    #frameset = [depth_frame, pose_frame]
    depth_frame = frameset[0]
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    depth_mat = np.asarray(depth_frame.get_data())
    
    pose_frame = frameset[1]
    points = scan_points(depth_mat, depth_intrin, depth_scale)
    r_angles, trans_vec = get_pose(pose_frame)

    rot_func = R.from_quat(r_angles, normalized=None)#R.from_euler('xyz', r_angles, degrees=True)
    roti = rot_func.apply(points)
    
    trans_points = np.zeros_like(roti)
    np.add(roti, trans_vec, trans_points)

    return trans_points, trans_vec

def map_func():
    pipe_list, DEPTH_SCALE = get_pipes()
    print("{} pipes created!".format(len(pipe_list)))
    map_point_cloud = np.zeros((1, 3), np.float32)
    
    decimate = rs.decimation_filter()

    while True:
        frameset = get_frames(pipe_list, decimate) #get the depth and pose frames from the camera stream
        points, current_pos = translate_frame_to_points(frameset, DEPTH_SCALE)

        print("points_shape: ", points.shape)
        if map_point_cloud.shape[0] >= MAX_POINT_COUNT:
            print ("They don't wanna see me in the ends")
            map_point_cloud = map_point_cloud[(points.shape[0] - 1):-1]
        
        map_point_cloud = np.append(map_point_cloud, points, axis=0) #append the points to the overall map

        portrait = scan_portrait(map_point_cloud, current_pos)
        
        cv2.imshow("circumstances", portrait)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()



def main():
    print("Accumulative Scanning Map")
    print("Still under construction...")

    map_func()
    

if __name__ == "__main__":
    main()