import numpy as np
import pyrealsense2 as rs
import cv2

from scipy.spatial.transform import Rotation as R
from scipy.ndimage import convolve

class scan_portait:
	#This makes scan portrait a unified object to minimize repetition of code and increase understandability

	def __init__(self, portrait_width=500, meter_width=8, max_point_count=0.5e4):
		self.portrait_width = portrait_width
		self.meter_width = meter_width
		self.scale = portrait_width / meter_width

		self.points = np.zeros((1, 3), dtype=np.float32)
		self.MAX_POINT_COUNT = max_point_count
		
		self.portrait = None

		self.rotation_quat = None
		self.translation = None


	
	def conv_reduce(self, reduction_coeff=0.2):
		#reduce the current portrait down to a given output size given the
		# NOTE: formula for output width: (W−F+2P)/S+1 
		# NOTE: to reduce without any weird overlapping, step by the same size as kernel size

		#NOTE: default reduction is set to 1/5 of the original size but can be changed as necessary
		output_w = self.portrait_width * reduction_coeff

		#Reduction by a certain coefficient should only work if the original width divides evenly into it
		assert (output_w % 1) == 0
		
		kernel_size = int(self.portrait_width / output_w)
		kernel = np.full((kernel_size, kernel_size), fill_value=(1/255))

		print ("kernel: ", kernel.shape)

		red_portrait = convolve(self.portrait, weights=kernel, mode='constant', cval=0.0)
		
		#This assert is here for debug to make sure that the output vector is the right shape
		print ("res_portrait: ", red_portrait.shape)
		assert red_portrait.shape[1] == int(output_w)
		
		self.portrait = red_portrait
		self.portrait_width = output_w

		return red_portrait

	def add_points(self, new_points):
		#add the new points to the larger numpy array of points
		#new points is a [n, 3] numpy array

		#trim the array if it gets too large
		if self.points.shape[0] >= self.MAX_POINT_COUNT:
			self.points = self.points[new_points.shape[0]: -1]

		self.points = np.append(self.points, new_points, axis=0)
		
	def xy_to_coords(self, coord_pair):
		#returns the indices on a portrait that correspond to a certain point on the portrait
		#coord pair is a 2 numpy array [x, y]
		center_val = int(self.portrait_width / 2) #this may be rough but it should work
		delta_x = int(self.scale * coord_pair[0])
		delta_y = int(self.scale * coord_pair[1])

		return (delta_x + center_val), (self.portrait_width - (delta_y + center_val))

	def coords_to_xy(self, ord_vec):
		#convert a vector of pixel coordinates to real world coordinates

		center_val = int(self.portrait_width / 2)
		ord_vec[:, 1] -= center_val
		ord_vec[:, 0] -= (self.portrait_width - 1) - center_val 
		ord_vec[:, 0] = -1 * ord_vec[:, 0]

		print ("ord_vec: ", ord_vec.shape)
		print (ord_vec.dtype)
		ord_vec = np.divide(ord_vec, self.scale)

		temp = ord_vec.copy()
		ord_vec[:, 0] = temp[:, 1]
		ord_vec[:, 1] = temp[:, 0]
		# ord_vec[:, 1] = ord_vec[:, 0]

		print (ord_vec)
		return ord_vec


	def get_portrait_from_frames(self, frameset, depth_scale):
		#return the scan portrait based on the depth and pose frames
		#and updates the class's potrait value

		points, _ = self.translate_frame_to_points(frameset, depth_scale)
		self.add_points(points)

		self.portrait = self.scan_portrait(self.points, panel_width=self.portrait_width)
		
		return self.portrait

	def get_portrait(self):
		return self.portrait
	
	def get_pose_from_frame(self, pose_frame):
		#return pose and translation as a 6-long numpy array
		#Also, update the pose within the class

		if not pose_frame:
			print("no pose found!")
			return np.asarray([], np.float32), np.asarray([], np.float32)
		
		data = pose_frame.get_pose_data()
		#Note: Rotation in returned as a quaternion
		self.rotation_quat = np.asarray([-data.rotation.x, -data.rotation.y, -data.rotation.z, data.rotation.w], np.float32)
		self.translation = np.asarray([data.translation.x, data.translation.y, data.translation.z], np.float32)
		
		return self.rotation_quat, self.translation
	
	def get_pose(self):
		return self.rotation_quat, self.translation

	def scan_portrait(self, points, min_height = -0.3, max_height = 0.3, panel_width = 1000):
		#min height and max height are minimum and maximum heights that will be included in the scan portrait
		#output a numpy matrix that is effectively a slice of the points that are provided

		portrait_mat = np.zeros((self.portrait_width, self.portrait_width), np.uint8)  

		center = int(self.portrait_width / 2)

		min_cond = points[:, 1] >= min_height
		max_cond = points[:, 1] <= max_height

		truth_cond = np.logical_and(min_cond, max_cond)

		height_valid_points = points[truth_cond]
		#print("height_valid_points shape: ", height_valid_points.shape)
		screen_points = np.delete(height_valid_points, 1, axis=1)

		screen_points = (screen_points * self.scale) + center - 1
		x_cond = np.logical_and(screen_points[:, 0] >= 0, screen_points[:, 0] < panel_width)
		y_cond = np.logical_and(screen_points[:, 1] >= 0, screen_points[:, 1] < panel_width)

		write_cond = np.logical_and(x_cond, y_cond)
		screen_points = screen_points[write_cond]

		screen_coords = screen_points.astype(np.int32)

		#print("points shape: ", points.shape)
		#print("screen_coords shape: ", screen_coords.shape)
		screen_coords[:, 1] = -screen_coords[:, 1] + self.portrait_width - 1
		portrait_mat[screen_coords[:, 1], screen_coords[:, 0]] = 255

		return portrait_mat

	def translate_frame_to_points(self, frameset, depth_scale):
		#returns the points rotated and translated to absolute coordinates (relative to where the program started)
		#Note: add the translation between the depth camera and the translation center for the current holder
		depth_frame = frameset[0]
		depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
		depth_mat = np.asarray(depth_frame.get_data())
	
		pose_frame = frameset[1]
		points = self.scan_points(depth_mat, depth_intrin, depth_scale)
		r_angles, trans_vec = self.get_pose_from_frame(pose_frame)

	
		rot_func = R.from_quat(r_angles, normalized=None)
		roti = points #rot_func.apply(points)
	
		trans_points = points #np.zeros_like(roti)
		#np.add(roti, trans_vec, trans_points)

		return trans_points, trans_vec

	def scan_points(self, image_frame, intren, depth_scale):
		#Move through one slice of the depth image and store only a single slice
		scan_row = int(image_frame.shape[0] / 2)
		row_size = image_frame.shape[1]
	
		point_collection = []
		for j in range(row_size):
			depth_val = image_frame[scan_row][j]
			point = rs.rs2_deproject_pixel_to_point(intren, [j, scan_row], depth_val)
			point = np.asarray(point) * depth_scale
			point_collection.append(point)

		return np.asarray(point_collection, np.float32)

	def vector_from_vel(self, x_vel, y_vel):
		#this will take the x and y velocities from the pupper's user_controller and rotate that vector to be in the original reference frame
		rot_func = R.from_quat(self.rotation_quat)
		#TODO: update to include YAW rate, or maye not
		ref_vec = np.array([-y_vel, 0, x_vel])
		rotated_vec = ref_vec#rot_func.apply(ref_vec)

		return np.delete(rotated_vec, 1, axis=0)



