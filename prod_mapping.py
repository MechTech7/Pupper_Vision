import numpy as np
import cv2
import pyrealsense2 as rs

import image_scan as im_scan
import camera_management as c_man

def vector_test(scan_por, dir_vec, position, pix_scale=0.1):
	#TODO: Make this function mean something    
	#tests a vector in a given direction
	#all multi-element values are numpy arrays of type int
	# vector form: [x, y]
	# position form: [x, y] float32
	# the function will step along the vector N times by incrementing position by x and y

	portrait = scan_por.get_portrait()
	
	scan_por.xy_to_coords(self, [0, 0])

	n_count = 10
	thresh = 0.6

	score = 0

	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)
	for i in range(1, n_count + 1):
		test_point = position + i * dir_vec

		#conversion into occupancy map coords
		x_val, y_val = scan_por.xy_to_coords(self, test_point)
		sample = portrait[y_val, x_val]
		print("########Testing: " + str(y_val) + " " + str(x_val))

		if sample != 0:
			print("hotter than ever")
			score += 1
		
	line_end = center_pos + n_count * dir_vec
	rgb_portrait = cv2.cvtColor(portrait, cv2.COLOR_GRAY2RGB)
	cv2.line(rgb_portrait, (p_width - 1 - center_pos[1], center_pos[0]), (p_width - 1 - line_end[1], line_end[0]), (255, 0, 0), 2)

	percent = score / n_count
	return ((percent - thresh) > 0), rgb_portrait

def main():
	cam_pipes, depth_scale = c_man.get_pipes()
	print("{} pipes created!".format(len(cam_pipes)))

	dec_filter = rs.decimation_filter()

	
	dep_scan = im_scan.scan_portait()
	
	
	while True:
		frameset = c_man.get_frames(cam_pipes, dec_filter)
		scan_mat = dep_scan.get_portrait_from_frames(frameset, depth_scale)
		
		cv2.imshow("circumstances", scan_mat)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cv2.destroyAllWindows()


	
if __name__ == '__main__':
	main()	

