import pyrealsense2 as rs

def get_frames(pipe_list, filter=None):
	depth_frame = None
	pose_frame = None

	count = 0
	for pipe_pair in pipe_list:
		pipe = pipe_pair[0]
		type_val = pipe_pair[1]

		#print("trying pipe: ", count)

		count += 1
		frames = pipe.wait_for_frames()

		
		if type_val == 'd':
			#if its a depth frame...
			#print("depth frame recieved")
			depth_frame = frames.get_depth_frame()

			if filter != None:
				print ("decimating!")
				depth_frame = filter.process(depth_frame)
			continue


		if type_val == 'p':
			#if its a pose frame
			#print("pose frame recieved") 
			pose_frame = frames.get_pose_frame()

		else:
			print("Note: neither depth or pose frames available on this pipe (what's it even for?)")

	return [depth_frame, pose_frame]

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