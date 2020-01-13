import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import cv2

def get_depth_scale(pipe):
    #this method procures the scale constant to convert depth from raw images to meters
    profile = pipe.get_active_profile()
    dev = profile.get_device() #right now, D435i is the only device so this works fine
    scale = dev.first_depth_sensor().get_depth_scale()
    return scale

##Note: it is difficult to convert from 3D-space into 2D image that works well as a representation
#TODO: work on inferencing on this data
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

        

#Blob detection is too "intelligent", there are too many parameters to optimize especially for a proof-of-concept
#A sligtly simpler idea is that we can convert the camera's depth camera into a simplified top-down image like a laser scanner would produce and then process from there.
#This would be much less data to work with
pipe = rs.pipeline()
pipe.start()

hole_fill = rs.hole_filling_filter()

DEPTH_SCALE = get_depth_scale(pipe)

#scan the middle line in the depth image

while True:
    frames = pipe.wait_for_frames()
    
    depth_frame = frames.get_depth_frame()
    #depth_frame = hole_fill.process(depth_frame)
    
    depth_image = np.asarray(depth_frame.get_data())
    
    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
    scan_image = frame_to_scan(depth_image, depth_intrin, DEPTH_SCALE)
    
    depth_image = 100 * depth_image * DEPTH_SCALE
    depth_image = depth_image.astype(np.uint8)
    
    
    rgb_depth_img = cv2.cvtColor(depth_image, cv2.COLOR_GRAY2RGB)
    line_thickness = 2
    print("image shape: ", rgb_depth_img.shape)
    cv2.line(rgb_depth_img, (0, int(rgb_depth_img.shape[0] / 2)), (rgb_depth_img.shape[1] - 1, int(depth_image.shape[0] / 2) ), (0, 255, 0), line_thickness)

    cv2.imshow('scan_image', scan_image)
    cv2.imshow('depth_image', rgb_depth_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Depth scale acquired: ", DEPTH_SCALE)
        break

cv2.destroyAllWindows()