import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
import math as m
import cv2

#Outline for this program:  
#This formalizes the occupancy map that was loosly defined in the pervious mapping programs

MAX_POINT_COUNT = 10e3 #maximum number of points that can be held in the point map
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
	quat_arr = np.asarray([-data.rotation.x, -data.rotation.y, -data.rotation.z, data.rotation.w], np.float32)
	

	return quat_arr, np.asarray([data.translation.x, data.translation.y, data.translation.z], np.float32)

def scan_portrait(points, current_pos, min_height = -0.2, max_height = 0.2, panel_width = 1000):
	#min height and max height are minimum and maximum heights that will be included in the scan portrait
	#output a numpy matrix that is effectively a slice of the points that are provided

	MAX_WIDTH = 8

	#Note: this display is O(N) so its probably really slow.  
	portrait_mat = np.zeros((panel_width, panel_width), np.uint8)  

	scale_val = (panel_width - 10) / (MAX_WIDTH)

	center = int(panel_width / 2)

	print("pos shape: ", current_pos.shape)

	#points = np.add(points, current_pos)
	min_cond = points[:, 1] >= min_height
	max_cond = points[:, 1] <= max_height

	truth_cond = np.logical_and(min_cond, max_cond)

	height_valid_points = points[truth_cond]
	print("height_valid_points shape: ", height_valid_points.shape)
	screen_points = np.delete(height_valid_points, 1, axis=1)

	screen_points = (screen_points * scale_val) + center - 1
	x_cond = np.logical_and(screen_points[:, 0] >= 0, screen_points[:, 0] < panel_width)
	y_cond = np.logical_and(screen_points[:, 1] >= 0, screen_points[:, 1] < panel_width)

	write_cond = np.logical_and(x_cond, y_cond)
	screen_points = screen_points[write_cond]

	screen_coords = screen_points.astype(np.int32)

	print("points shape: ", points.shape)
	print("screen_coords shape: ", screen_coords.shape)
	screen_coords[:, 1] = -screen_coords[:, 1] + panel_width - 1
	portrait_mat[screen_coords[:, 1], screen_coords[:, 0]] = 255 #problem: This Numpy indexing is not very straightforward [i, j] indices return a slice

	return portrait_mat


def old_scan_points(image_frame, intren, depth_scale):
	scan_row = int(image_frame.shape[0] / 2)
	row_size = image_frame.shape[1]
	
	point_collection = []
	for j in range(row_size):
		depth_val = image_frame[scan_row][j]
		point = rs.rs2_deproject_pixel_to_point(intren, [j, scan_row], depth_val)
		point = np.asarray(point) * depth_scale
		point_collection.append(point)

	return np.asarray(point_collection, np.float32)

def scan_points(depth_frame, d_height, thresh=0.2):
	pc = rs.pointcloud()
	points = pc.calculate(depth_frame)
	og_verts = points.get_vertices()
	print("og_verts: ", og_verts)

	verts = np.asanyarray(og_verts)

	print("verts: ", verts)
	print("obj_type: ", np.dtype(verts[0]))
	print ("verts shape: ", verts.shape)

	print("test_pr: ", verts[['f0', 'f1', 'f2']])
	x = np.asarray(verts[['f0']],dtype=np.float32)
	y = np.asarray(verts[['f1']],dtype=np.float32)
	z = np.asarray(verts[['f2']],dtype=np.float32)
	
	print("x: ", x)

	
	full = np.column_stack((x, y, z))
	full = full.astype(np.float32)

	print("full: ", full)
	print("full shape: ", np.shape(full))

	#get only points that are around a certain height
	low_cond = full[:, 1] - d_height >= -thresh
	high_cond = full[:, 1] - d_height <= thresh

	truth_cond = np.logical_and(low_cond, high_cond)
	return full[truth_cond]


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
	points = old_scan_points(depth_mat, depth_intrin, depth_scale)
	#points = scan_points(depth_frame, 0)
	r_angles, trans_vec = get_pose(pose_frame)

	
	rot_func = R.from_quat(r_angles, normalized=None)#R.from_euler('xyz', r_angles, degrees=True)
	roti = rot_func.apply(points)
	
	trans_points = np.zeros_like(roti)
	np.add(roti, trans_vec, trans_points)

	return trans_points, trans_vec

def new_translate(frameset, depth_scale):
		#returns the points rotated and translated
	#Note: add the translation between the depth camera and the translation center for the current holder
	#frameset = [depth_frame, pose_frame]
	depth_frame = frameset[0]
	depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
	depth_mat = np.asarray(depth_frame.get_data())
	
	pose_frame = frameset[1]
	points = old_scan_points(depth_mat, depth_intrin, depth_scale)
	#points = scan_points(depth_frame, 0)
	r_angles, trans_vec = get_pose(pose_frame)
	points = np.add(points, trans_vec)

	rot_func = R.from_quat(r_angles, normalized=None)#R.from_euler('xyz', r_angles, degrees=True)
	roti = rot_func.apply(points)
	

	return roti, trans_vec


def vector_test(portrait, dir_vec, position, pix_scale=0.1, ):
	#tests a vector in a given direction
	#all multi-element values are numpy arrays of type int
	# vector form: [x, y]
	# position form: [x, y] float32
	# the function will step along the vector N times by incrementing position by x and y

	p_width = np.shape(portrait)[1]
	center_val = int(p_width / 2)
	
	MAX_WIDTH = 8
	scale_val = (p_width - 10) / (MAX_WIDTH)

	dir_vec[1] = -dir_vec[1] #invert y

	center_pos = (scale_val * position).astype(np.int32) + center_val #define the origin of the vector

	n_count = 20
	max_safe_steps = 5

	obstacle_detected = False
	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)
	for i in range(1, n_count + 1):
		test_point = center_pos + 2 * i * dir_vec

		#conversion into occupancy map coords
		sample = portrait[p_width - 1 - test_point[1], test_point[0]]
		print("########Testing: " + str(test_point[0]) + " " + str(p_width - 1 - test_point[1]))

		if sample != 0 and i < max_safe_steps:
			print("hotter than ever")
			obstacle_detected = True
		
	line_end = center_pos + n_count * dir_vec
	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)

	
	cv2.line(rgb_portrait, (p_width - 1 - center_pos[1], center_pos[0]), (p_width - 1 - line_end[1], line_end[0]), (255, 0, 0), 2)
	cv2.circle(rgb_portrait, (p_width - 1 - line_end[1], line_end[0]), radius=2, color=(0, 255, 0))
	
	return obstacle_detected, rgb_portrait
	
def map_func():
	pipe_list, DEPTH_SCALE = get_pipes()
	print("{} pipes created!".format(len(pipe_list)))
	map_point_cloud = np.zeros((1, 3), np.float32)
	
	decimate = rs.decimation_filter()

	while True:
		frameset = get_frames(pipe_list, decimate) #get the depth and pose frames from the camera stream
		#points, current_pos = new_translate(frameset, DEPTH_SCALE)
		points, current_pos = translate_frame_to_points(frameset, DEPTH_SCALE)
		print("points_shape: ", points.shape)
		if map_point_cloud.shape[0] >= MAX_POINT_COUNT:
			print ("-------------They don't wanna see me in the ends")
			map_point_cloud = map_point_cloud[(points.shape[0] - 1):-1]
		
		map_point_cloud = np.append(map_point_cloud, points, axis=0) #append the points to the overall map

		portrait = scan_portrait(map_point_cloud, current_pos)
		
		#Vector testing section-----------------------
		test_vec = np.array([8, -8], np.int32)
		int_pos = current_pos.astype(np.int32)
		int_pos = np.delete(int_pos, 1, axis=0)

		test_val, rgb_portrait = vector_test(portrait, test_vec, np.delete(current_pos, 1, axis=0))
		
		print ("Test val: ", test_val)
		if (test_val):
			print("----------------watch out!")

		

		line_end = int_pos + 10 * test_vec

		print("int_pos: ", int_pos)
		print("line_end: ", line_end)
		
		cv2.imshow("circumstances", rgb_portrait)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cv2.destroyAllWindows()



def main():
	print("Accumulative Scanning Map")
	print("Still under construction...")

	map_func()
	

if __name__ == "__main__":
	main()