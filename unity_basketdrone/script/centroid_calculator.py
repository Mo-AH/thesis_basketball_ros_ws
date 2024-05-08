#!/usr/bin/env python3

import rospy
import time
import statistics
from vision_msgs.msg import Detection2D, Detection2DArray,  ObjectHypothesisWithPose
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

class CentroidCalculatorNode:
	def __init__(self):

		# Publisher and subscribers
		self.position_sub = rospy.Subscriber("drone_position", Point, self.position_cb, queue_size=1)
		self.detection_sub = rospy.Subscriber("detection_result", Detection2DArray, self.detection_cb, queue_size=1)
		self.centroid_pixel_pub = rospy.Publisher("centroid_pixel", Point, queue_size=1)
		self.centroid_pub = rospy.Publisher("centroid_position", Point,queue_size=1)
		self.centroid_distance_pub = rospy.Publisher("centroid_distance", Float32, queue_size=1)

		# Drone
		self.drone_position = Point()
		self.drone_position.x = -14.05
		self.drone_position.z = 7.55
		self.drone_position.y = rospy.get_param("~drone_altitude", 7)

		# Centroid and camera parameters
		self.centroid_method = rospy.get_param("~centroid_method",1)
		self.centroid_buffer = []
		self.image_center_x = 480
		self.image_center_y = 360
		if self.drone_position.y == 7:
			self.cameraview_half_z = 7.8
			self.cameraview_half_x = 5.8
		elif self.drone_position.y == 10:
			self.cameraview_half_z = 11.2
			self.cameraview_half_x = 8.4
		elif self.drone_position.y == 14:
			self.cameraview_half_z = 15.6
			self.cameraview_half_x = 11.7

		# Testing variables
		self.n_frames = 0
		self.centroid_distance_array = []
		self.n_failures = 0
		self.last_centroid = None
		self.max_speed = 0
		self.speed_buffer = []
		self.n_objects_buffer = []
		self.latency_times = []


# --------------------------------------

	# Updates the drone position
	def position_cb(self, data):
		self.drone_position = data


# --------------------------------------
	
	# Callback to YOLO detections:
	#	- computes the centroid (pixel) given the detections
	# 	- computes the centroid (position)
	#	- prints testing results
	def detection_cb(self, data):
		start_time = time.time()
		self.n_frames+=1
		
		total_detections = len(data.detections)
		self.n_objects_buffer.append(total_detections)

		# Compute centroid and append it in the buffer
		if total_detections > 0:
			if self.centroid_method == 1:
				centroid_x, centroid_y, ball_detected = self.get_pixel_centroid(data.detections)
			else:
				centroid_x, centroid_y, ball_detected = self.get_pixel_centroid_v2(data.detections)

			if len(self.centroid_buffer) > 3:
				self.centroid_buffer.pop(0)
			self.centroid_buffer.append((centroid_x, centroid_y))
		else:
			return

		# Compute and publish buffer centroid (pixel and positional)
		if self.centroid_buffer:
			avg_centroid_x = sum(x for x, _ in self.centroid_buffer) / len(self.centroid_buffer)
			avg_centroid_y = sum(y for _, y in self.centroid_buffer) / len(self.centroid_buffer)
			centroid = Point()
			centroid.z = 0
			if (ball_detected):
				centroid.x = centroid_x
				centroid.y = centroid_y
			else:
				centroid.x = avg_centroid_x
				centroid.y = avg_centroid_y
			self.centroid_pixel_pub.publish(centroid)
			self.compute_centroid_position(avg_centroid_x, avg_centroid_y)

		# TEST
		if self.n_frames%600 == 0:
			self.print_logs()
		#end_time = time.time()
		#latency = end_time - start_time
		#if self.n_frames < 100:
			#print(f"[CC] Latenza frame {self.n_frames}: {latency*1000.0:.5f} ms")  # Stampa la latenza
		#else:
		#	self.latency_times.append(latency)
		#	if len(self.latency_times) % 100 == 0:
		#		average_latency = sum(self.latency_times) / len(self.latency_times)
				#print(f"[CC] Media latenza: {average_latency*1000.0:.5f} ms su {len(self.latency_times)} callback")

# --------------------------------------
	

	# Converts the centroid in position coordinates,
	# publishes it and the distance drone-centroid (meters)
	def compute_centroid_position(self, centroid_x_pixel, centroid_y_pixel):
		x_shift = (centroid_y_pixel-self.image_center_y)/self.image_center_y * self.cameraview_half_z
		z_shift = (centroid_x_pixel-self.image_center_x)/self.image_center_x * self.cameraview_half_x
		centroid = Point()
		centroid.x = self.drone_position.x + x_shift
		centroid.y = 0
		centroid.z = self.drone_position.z + z_shift
		self.centroid_pub.publish(centroid)
		centroid_distance = Float32()
		centroid_distance.data = (x_shift**2 + z_shift**2) ** 0.5
		self.centroid_distance_pub.publish(centroid_distance)

		if self.last_centroid is not None:
			speed = (((centroid.x - self.last_centroid.x)**2 + (centroid.z -self.last_centroid.z)**2)**0.5) *10
			self.speed_buffer.append(speed)
			if speed > self.max_speed:
				self.max_speed = speed

		self.last_centroid = centroid

		# TEST
		self.centroid_distance_array.append(centroid_distance.data)

# --------------------------------------
	
	# Given the yolo detections, computes the coordinates (XY pixel) of the centroid.
	# The logic is to do a weighted average of the ball(0.8) and the 4 players(0.2) closer to the ball
	# if the ball is not detected, it does an average between the 4 players closer to last centroid
	def get_pixel_centroid(self,detections):
		centroid_x = 0.0
		centroid_y = 0.0
		players = []
		ball_detected = False

		# Check if the ball has been detected
		for detection in detections:
			if detection.results[0].id == 0:
				ball_x = detection.bbox.center.x
				ball_y = detection.bbox.center.y
				ball_detected = True
				break

		# TEST
		if not ball_detected:
			self.n_failures+=1

		# Compute a sorted list of the players with their distance to the ball/last centroid
		for detection in detections:
			if detection.results[0].id == 1:  # Giocatori
				if ball_detected:
					distance = ((detection.bbox.center.x - ball_x) ** 2 +
								(detection.bbox.center.y - ball_y) ** 2) ** 0.5
				elif len(self.centroid_buffer) > 0:
					distance = ((detection.bbox.center.x - self.centroid_buffer[-1][0]) ** 2 +
								(detection.bbox.center.y - self.centroid_buffer[-1][1]) ** 2) ** 0.5
				else:
					distance = ((detection.bbox.center.x - self.image_center_x) ** 2 +
								(detection.bbox.center.y - self.image_center_y) ** 2) ** 0.5
				players.append({
					'x': detection.bbox.center.x,
					'y': detection.bbox.center.y,
					'distance': distance
				})
		players_sorted = sorted(players, key=lambda player: player['distance'])

		# Consider just the 4 nearest players
		closest_players = players_sorted[:min(4, len(players_sorted))]

		# Compute the centroid
		if ball_detected and len(closest_players) > 0:
			weighted_ball_x = ball_x * 0.8
			weighted_players_x = sum(player['y'] for player in closest_players) * 0.2 / len(closest_players)
			centroid_x = weighted_ball_x + weighted_players_x

			weighted_ball_y = ball_y * 0.8
			weighted_players_y = sum(player['y'] for player in closest_players) * 0.2 / len(closest_players)
			centroid_y = weighted_ball_y + weighted_players_y
		elif ball_detected:
			centroid_x = ball_x
			centroid_y = ball_y
		elif len(closest_players) > 0:
			centroid_x = sum(player['x'] for player in closest_players) / len(closest_players)
			centroid_y = sum(player['y'] for player in closest_players) / len(closest_players)
		else:
			centroid_x = self.image_center_x
			centroid_y = self.image_center_y

		return centroid_x, centroid_y, ball_detected


# --------------------------------------
	
	# Given the yolo detections, computes the coordinates (XY pixel) of the centroid.
	# The logic is to do a weighted average of the ball(0.8) and the 4 players(0.2) closer to the ball
	# if the ball is not detected, it does an average between the 4 players closer to last centroid
	def get_pixel_centroid_v2(self,detections):
		centroid_x = 0.0
		centroid_y = 0.0
		players = []
		ball_detected = False

		# Check if the ball has been detected
		for detection in detections:
			if detection.results[0].id == 0:
				ball_x = detection.bbox.center.x
				ball_y = detection.bbox.center.y
				ball_detected = True
				break

		# TEST
		if not ball_detected:
			self.n_failures+=1

		# Compute a sorted list of the players with their distance to the ball/last centroid
		for detection in detections:
			if detection.results[0].id == 1:  # Giocatori
				if ball_detected:
					distance = ((detection.bbox.center.x - ball_x) ** 2 +
								(detection.bbox.center.y - ball_y) ** 2) ** 0.5
				elif len(self.centroid_buffer) > 0:
					distance = ((detection.bbox.center.x - self.centroid_buffer[-1][0]) ** 2 +
								(detection.bbox.center.y - self.centroid_buffer[-1][1]) ** 2) ** 0.5
				else:
					distance = ((detection.bbox.center.x - self.image_center_x) ** 2 +
								(detection.bbox.center.y - self.image_center_y) ** 2) ** 0.5
				players.append({
					'x': detection.bbox.center.x,
					'y': detection.bbox.center.y,
					'distance': distance
				})
		players_sorted = sorted(players, key=lambda player: player['distance'])

		# Consider just the 4 nearest players
		closest_players = players_sorted[:min(8, len(players_sorted))]

		# Compute the centroid
		if ball_detected:

			weighted_ball_x = ball_x * 0.5
			weighted_players_x = sum(player['y'] for player in closest_players) * 0.5 / len(closest_players)
			centroid_x = weighted_ball_x + weighted_players_x
			
			weighted_ball_y = ball_y * 0.5
			weighted_players_y = sum(player['y'] for player in closest_players) * 0.5 / len(closest_players)
			centroid_y = weighted_ball_y + weighted_players_y

		else:
			centroid_x = sum(player['x'] for player in closest_players) / len(closest_players)
			centroid_y = sum(player['y'] for player in closest_players) / len(closest_players)

		return centroid_x, centroid_y, ball_detected

# --------------------------------------
	
	def print_logs(self):
		if self.n_frames == 0:
			print("No frames processed.")
			return

		# Calcola il picco e il RMS dell'errore del centroide
		picco_errore = max(self.centroid_distance_array)
		rms_errore = (sum(x**2 for x in self.centroid_distance_array) / len(self.centroid_distance_array))**0.5
		average_speed = sum(self.speed_buffer)/len(self.speed_buffer)
		n_objects = sum(self.n_objects_buffer)/len(self.n_objects_buffer)
		std_dev = statistics.stdev(self.n_objects_buffer)
		fail = ((self.n_frames-self.n_failures)/self.n_frames)*100
		# Stampa i risultati
		
		print(f"Average centroid speed: {average_speed:.2f} m/s")
		print(f"Max centroid speed: {self.max_speed:.2f} m/s")
		print(f"Error RMS: {rms_errore:.2f}")
		print(f"Error peak: {picco_errore:.2f}")
		print(f"Ball not detected: {fail:.2f} %")
		print(f"Mean objects detected: {n_objects:.2f}")
		print(f"Stdev objects detected: {std_dev:.2f}")



# --------------------------------------

if __name__ == "__main__":
	rospy.init_node("centroid_calculator")
	node = CentroidCalculatorNode()
	print('NODE CENTROID CALCULATOR STARTED')
	rospy.spin()
