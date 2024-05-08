#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
import time


class DroneControllerNewNode:
	def __init__(self):
		# Publisher/Subscribers
		self.position_sub = rospy.Subscriber("drone_position", Point, self.position_cb, queue_size=1)
		self.centroid_sub = rospy.Subscriber("centroid_pixel", Point, self.centroid_cb,queue_size=1)
		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		
		# Speed
		self.linear_speed_x = 0.2
		self.linear_speed_y = 0.2
		self.speed_limit = 20
		
		# Camera
		self.image_center_x = 480
		self.image_center_y = 360
		box_method = rospy.get_param("~box_method", 1)
		if box_method == 1:
			self.x_box = 100
		elif box_method == 2:
			self.x_box = 300
		else:
			self.x_box = 500

		self.y_box = self.x_box * 3/4

		# Drone 
		self.drone_position = Point()
		self.drone_position.x = -14.05
		self.drone_position.z = 7.55
		self.drone_position.y = rospy.get_param("~drone_altitude", 7)
		if self.drone_position.y == 7:
			self.zInf = 5
			self.zSup = 10
			self.xInf = -24
			self.xSup = -4
		elif self.drone_position.y == 10:
			self.zInf = 6
			self.zSup = 9
			self.xInf = -22
			self.xSup = -6
		elif self.drone_position.y == 14:
			self.zInf = 7
			self.zSup = 8
			self.xInf = -18
			self.xSup = -10

		#TEST
		self.n_frames = 0
		self.recentering = False
		self.num_recenter_events = 0
		self.num_recenter_frames = 0
		self.frame_time = 0.1 				# 10 Hz -> 0.1 seconds
		self.latency_times = []
		self.speed_buffer = []

# --------------------------------------
	
	# Updates the drone position
	def position_cb(self, data):
		self.drone_position = data

# --------------------------------------
	
	# Given the centroid (pixel coordinates), checks if the centroid has to be centered
	# and send velocity commands to the drone
	def centroid_cb(self, data):

		start_time = time.time()
		
		twist_msg = Twist()

		# Distance from the center of the camera
		x_distance = self.image_center_x - data.x
		y_distance = self.image_center_y - data.y


		# Decide if and in which direction it has to be centered (xy, x, y)
		x_box = self.x_box/2
		y_box = self.y_box/2
		if abs(x_distance) > x_box:
			if (x_distance) > 0:
				x_error = x_distance - x_box
			else:
				x_error = x_distance + x_box
		else:
			x_error = 0

		if abs(y_distance) > y_box:
			if (y_distance) > 0:
				y_error = y_distance - y_box
			else:
				y_error = y_distance + y_box
		else:
			y_error = 0


		# --------------------------------------

		# Compute the speed and saturate if needed
		if abs(x_error)>0:
			twist_msg.linear.z = -self.linear_speed_x * x_error 
		if abs(y_error)>0:
			twist_msg.linear.x = -self.linear_speed_y * y_error 

		if abs(twist_msg.linear.x) > self.speed_limit:
			twist_msg.linear.x = self.speed_limit if twist_msg.linear.x > 0 else -self.speed_limit
		if abs(twist_msg.linear.z) > self.speed_limit:
			twist_msg.linear.z = self.speed_limit if twist_msg.linear.z > 0 else -self.speed_limit

		# Azzera la velocità su x/z quando si trova in prossimità dei bordi (x:-28,0 - z:0, 15)
		if (twist_msg.linear.x < 0 and self.drone_position.x <= self.xInf) or \
			(twist_msg.linear.x > 0 and self.drone_position.x >= self.xSup):
			twist_msg.linear.x = 0
		if (twist_msg.linear.z < 0 and self.drone_position.z <= self.zInf) or \
			(twist_msg.linear.z > 0 and self.drone_position.z >= self.zSup):
			twist_msg.linear.z = 0

		# Pubblica il messaggio di velocità
		self.vel_pub.publish(twist_msg)

		# TEST --------------------------------
		
		self.n_frames += 1
		
		if abs(twist_msg.linear.z)>0 or abs(twist_msg.linear.x)>0:
			if not self.recentering:
				self.recentering = True
				self.num_recenter_events +=1
			self.num_recenter_frames +=1
		else:
			if self.recentering:
				self.recentering = False


		speed = ((twist_msg.linear.z**2 + twist_msg.linear.x**2)**0.5)
		self.speed_buffer.append(speed)



		if self.n_frames%600 == 0:
			self.print_logs()


		#end_time = time.time()
		#latency = (end_time - start_time)
		#if self.n_frames < 100:
			#print(f"[DC] Latenza frame {self.n_frames}: {latency*1000:.5f} ms")  # Stampa la latenza
		#else:
		#	self.latency_times.append(latency)
		#	if len(self.latency_times) % 100 == 0:
		#		average_latency = sum(self.latency_times) / len(self.latency_times)
				#print(f"[DC] Media latenza: {average_latency*1000:.5f} ms su {len(self.latency_times)} callback")



# --------------------------------------
	
	def print_logs(self):
		if self.n_frames == 0:
			print("No frames processed.")
			return

		# Calcola il tempo totale di ricentramento
		total_recenter_time = self.num_recenter_frames * 0.1

		# Calcola il tempo medio di ricentramento
		tempo_medio_recenter = 0.0 if self.num_recenter_events == 0 else total_recenter_time / self.num_recenter_events

		# Calcola la percentuale di tempo di ricentramento rispetto al tempo totale della simulazione
		percentuale_recenter_time = (self.num_recenter_frames  / self.n_frames) * 100.0

		average_speed = sum(self.speed_buffer)/len(self.speed_buffer)

		# Stampa i risultati
		print(f"\nNumero di frames processati: {self.n_frames}")
		print(f"Average drone speed: {average_speed:.2f} m/s")
		print(f"Recentering events: {self.num_recenter_events}")
		print(f"Average recentering time: ({percentuale_recenter_time:.3f} %)")

# --------------------------------------

if __name__ == "__main__":
	rospy.init_node("drone_controller_new")
	node = DroneControllerNewNode()
	print('NODE DRONE CONTROLLER STARTED')
	rospy.spin()