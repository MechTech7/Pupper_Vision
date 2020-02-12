import numpy as np
import cv2
import pyrealsense2 as rs

import image_scan as im_scan
import camera_management as c_man

def vector_test(scan_por, dir_vec, position):
	#tests a vector in a given direction
	#all multi-element values are numpy arrays of type int
	# vector form: [x, y]
	# position form: [x, y] float32
	# the function will step along the vector N times by incrementing position by x and y

	portrait = scan_por.get_portrait()
	scan_por.xy_to_coords(self, [0, 0])

	n_count = 10
	thresh = 0.6 #maximum distance in meters at which something is considered a credible obstacle

	sized_vec = dir_vec / 10

	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)

	#Draw a line representing the vector
	line_end = position + n_count * sized_vec
	end_x, end_y = scan_por.xy_to_coords(line_end)

	cv2.line(rgb_portrait, (end_x, end_y), color=(255, 0, 0), thickness=2)
	for i in range(1, n_count + 1):
		test_point = position + i * sized_vec

		#conversion into occupancy map coordinates
		x_val, y_val = scan_por.xy_to_coords(test_point)
		
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
	return ((percent - thresh) > 0), rgb_portrait

def main():
	cam_pipes, depth_scale = c_man.get_pipes()
	#print("{} pipes created!".format(len(cam_pipes)))

	dec_filter = rs.decimation_filter()

	
	dep_scan = im_scan.scan_portait()
	
	
	while True:
		frameset = c_man.get_frames(cam_pipes, dec_filter)
		scan_mat = dep_scan.get_portrait_from_frames(frameset, depth_scale)
		
		_, trans_position = dep_scan.get_pose()

		vector_test(dep_scan, dir_vec=np.array([3, 4]), position=trans_position)
		cv2.imshow("circumstances", scan_mat)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cv2.destroyAllWindows()


	
if __name__ == '__main__':
	main()	

