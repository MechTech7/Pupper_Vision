import numpy as np
import cv2
import pyrealsense2 as rs

import image_scan as im_scan
import camera_management as c_man
from UDP_trajectory_recv import Trajectory_Reciever
from UDPComms import Publisher

def soft_danger(norm_collection, min_dist=0.3, danger_dist=0.5):
	#take the points and minimum distance and construct a soft value danger (between 0 and 1)
	#norm_collection: the collection of the norms on the line
	#min_dist: minimum distance from the camera any point can be
	
	#mean_point_dist = np.mean(norm_collection, axis=0)
	min_point_dist = np.min(norm_collection, axis=0)

	x_trans = 
	if min_point_dist >= danger_dist:
		return 1.0
	elif min_point_dist <= min_dist:
		return 0.0
	
	return (danger_dist - min_dist) * (min_point_dist - min_dist)

def real_oord_test(scan_por, dir_vec, position, dist_threshold=0.3):
	#This a function that does the cosine similarity detection but with the coordinates from the scan portrait
	#converted into real coordinate values to preserve direction
	
	#dir_vec is a numpy array with [x, z]
	#position is a numpy array with [x, z]
	
	portrait = scan_por.get_portrait()
	print("position shape: ", position.shape)

	pdir_x, pdir_y = scan_por.xy_to_coords(dir_vec)

	pix_x, pix_y = scan_por.xy_to_coords(position)

	#pixel-coordinates of all of the points that are "on"
	on_points = np.argwhere(portrait)

	real_coords = scan_por.coords_to_xy(on_points)
	offset_points = real_coords - position

	print ("offset points: ", offset_points.shape)
	print("offset points[:0]" , offset_points[:, 0].shape)
	print ("offset points [:1]", offset_points[:, 1].shape)

	
	mul_vec = np.multiply(offset_points, dir_vec)

	dir_norm = np.linalg.norm(dir_vec)
	points_norm = np.linalg.norm(offset_points, axis=1)
	print("points norm:", points_norm.shape)

	norm_prod = points_norm * dir_norm
	norm_prod = np.expand_dims(norm_prod, axis=1)
	print("norm prod: ", norm_prod.shape)

	dot_prod = np.sum(mul_vec, axis=1)
	dot_prod = np.expand_dims(dot_prod, axis=1)

	print ("dot prod: ", dot_prod.shape)
	cos_sim = np.divide(dot_prod, norm_prod)

	print("cos sim: ", cos_sim.shape)

	sim_thresh = 0.9
	truth_mask = (cos_sim >= sim_thresh)
	truth_mask = np.ravel(truth_mask)

	print ("truth_mask: ", truth_mask.shape)
	

	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)

	dist_truth = points_norm <= dist_threshold

	big_mask = np.logical_and(truth_mask, dist_truth)

	good_points = offset_points[big_mask]
	# print(pdir_x, pdir_y)
	# print(pix_x-250+pdir_x * 10, pix_y-250+pdir_y * 10)
	cv2.line(rgb_portrait, (pix_x, pix_y), (pix_x-250+pdir_x, pix_y-250+pdir_y), color=(255, 0, 0), thickness=2)
	
	
	print ("hit points: ", good_points.shape)
	soft_d_val = soft_danger(points_norm, min_dist=dist_threshold)
	return soft_d_val, rgb_portrait

def cos_sim_test(scan_por, dir_vec, position):
	#There are a lot of flaws with this.  namely that converting the direction vector to pixel coordinates doesn't preserve the direction (it shifts it by a certain amount)
	
	portrait = scan_por.get_portrait()

	pdir_x, pdir_y = scan_por.xy_to_coords(dir_vec)

	pix_x, pix_y = scan_por.xy_to_coords(position)

	pos_arr = np.asarray([pix_y, pix_x])

	print("position shape: ", position.shape)

	#make sure points are in the right direction:
	on_points = np.argwhere(portrait)
	print ("argwhere shape: ", on_points.shape)

	#TODO: filter out points that aren't in the direction you want.
	
	offset_points = on_points - pos_arr
	print ("offset points: ", offset_points.shape)
	print("offset points[:0]" , offset_points[:, 0].shape)
	print ("offset points [:1]", offset_points[:, 1].shape)

	direc_arr = np.array([pdir_y, pdir_x])
	mul_vec = np.multiply(offset_points, direc_arr)

	dir_norm = np.linalg.norm(direc_arr)
	points_norm = np.linalg.norm(offset_points, axis=1)
	print("points norm:", points_norm.shape)
	norm_prod = points_norm * dir_norm
	norm_prod = np.expand_dims(norm_prod, axis=1)
	print("norm prod: ", norm_prod.shape)

	dot_prod = np.sum(mul_vec, axis=1)
	dot_prod = np.expand_dims(dot_prod, axis=1)

	print ("dot prod: ", dot_prod.shape)
	cos_sim = np.divide(dot_prod, norm_prod)

	print("cos sim: ", cos_sim.shape)

	sim_thresh = 0.9
	truth_mask = (cos_sim >= sim_thresh)
	truth_mask = np.ravel(truth_mask)

	print ("truth_mask: ", truth_mask.shape)
	good_points = offset_points[truth_mask]

	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)
	cv2.line(rgb_portrait, (pix_x, pix_y), (pix_x + pdir_x * 10, pix_y + pdir_y * 10), color=(255, 0, 0), thickness=2)
	
	print ("hit points: ", good_points.shape)
	return (good_points.shape[0] > 1), rgb_portrait

def new_vec_test(scan_por, dir_vec, position):
	#TODO: add distance thresholding to function
	#dir_vec is a numpy array with [x, z]
	#position is a numpy array with [x, z]

	portrait = scan_por.get_portrait()

	pdir_x, pdir_y = scan_por.xy_to_coords(dir_vec)

	pix_x, pix_y = scan_por.xy_to_coords(position)

	pos_arr = np.asarray([pix_y, pix_x])

	print("position shape: ", position.shape)

	#make sure points are in the right direction:
	on_points = np.argwhere(portrait)
	print ("argwhere shape: ", on_points.shape)

	#TODO: filter out points that aren't in the direction you want.
	
	offset_points = on_points - pos_arr
	print ("offset points: ", offset_points.shape)
	print("offset points[:0]" , offset_points[:, 0].shape)
	print ("offset points [:1]", offset_points[:, 1].shape)

	mul_vec = np.multiply(offset_points, np.array([pdir_y, pdir_x]))
	prop_dir = mul_vec >= 0 #get a truth mask for everywhere mul_vec has a positive value

	prop_dir = np.all(prop_dir, axis=1)
	print ("prop_dir: ", prop_dir.shape)
	
	offset_points = offset_points[prop_dir]
	slopes = np.divide(offset_points[:, 0], (offset_points[:, 1] + 1e-10)) 
	print ("slopes shape: ", slopes.shape)

	ref_slope = pdir_y / (pdir_x + 1e-10)

	sub_slopes = slopes - ref_slope
	sub_slopes = np.abs(sub_slopes)

	deviation = 0.1

	hit_idxs = np.argwhere(sub_slopes <= deviation)
	hit_points = on_points[hit_idxs]

	print ("hit points: ", hit_points.shape)
	

	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)
	
	

	cv2.line(rgb_portrait, (pix_x, pix_y), (pix_x + pdir_x * 10, pix_y + pdir_y * 10), color=(255, 0, 0), thickness=2)

	return (hit_points.shape[0] > 1), rgb_portrait

def vector_test(scan_por, dir_vec, position):
	#tests a vector in a given direction
	#all multi-element values are numpy arrays of type int
	# vector form: [x, y]
	# position form: [x, y] float32
	# the function will step along the vector N times by incrementing position by x and y

	portrait = scan_por.get_portrait()

	n_count = 10
	thresh = 0.5 #maximum distance in meters at which something is considered a credible obstacle
	
	min_perc = 0.1 #minimum percentage to be considered a "hit"

	sized_vec = dir_vec / 10

	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)

	#Draw a line representing the vector
	line_end = position + n_count * sized_vec
	start_x, start_y = scan_por.xy_to_coords(position)
	end_x, end_y = scan_por.xy_to_coords(line_end)
	cv2.line(rgb_portrait, (start_x, start_y), (end_x, end_y), color=(255, 0, 0), thickness=2)
	
	#Set accumulator to zero
	score = 0

	for i in range(1, n_count + 1):
		test_point = position + i * sized_vec

		#conversion into occupancy map coordinates
		x_val, y_val = scan_por.xy_to_coords(test_point)
		#TODO: do an in-bounds check on the xy of the scanning vector
		sample = portrait[y_val, x_val]
		print("----Test_point: [" + str(y_val) + ", " + str(x_val) + "]")

		point_dist = np.linalg.norm(test_point)
		print("----Real distance: ", str(point_dist))

		if sample != 0 and point_dist <= thresh:
			print("I was in france")
			score += 1

		#draw circles over the image
		cv2.circle(rgb_portrait, (x_val, y_val), radius=2, color=(0, 255, 0), thickness=1)
	
	percent = score / n_count
	print ("percent value: ", percent)
	
	return ((percent - thresh) >= min_perc), rgb_portrait

def main():
	cam_pipes, depth_scale = c_man.get_pipes()
	#print("{} pipes created!".format(len(cam_pipes)))

	dec_filter = rs.decimation_filter()
	dep_scan = im_scan.scan_portait()

	traj_recv = Trajectory_Reciever()
	stay_go_sender = Publisher(3500)
	
	while True:
		frameset = c_man.get_frames(cam_pipes, dec_filter)
		scan_mat = dep_scan.get_portrait_from_frames(frameset, depth_scale)
		
		#dep_scan.conv_reduce()
		_, trans_position = dep_scan.get_pose()
		xz_pos = np.delete(trans_position, 1, axis=0)

		x_vel, y_vel = traj_recv.get_trajectory()
		control_vec = dep_scan.vector_from_vel(x_vel, y_vel)

		print ("!dir_vec: ", control_vec)
		soft_danger, rgb_mat = real_oord_test(dep_scan, dir_vec=control_vec, position=xz_pos)
		
		stay_go_sender.send({"soft_danger": soft_danger})
		print("-------------------------pass_fail: ", soft_danger)
		cv2.imshow("circumstances", rgb_mat)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cv2.destroyAllWindows()


	
if __name__ == '__main__':
	main()	

