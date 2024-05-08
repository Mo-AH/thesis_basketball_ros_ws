#!/usr/bin/env python3

import rospy
from vision_msgs.msg import Detection2D, Detection2DArray,  ObjectHypothesisWithPose
from geometry_msgs.msg import Twist, Point

class DroneControllerNode:
	def __init__(self):
		detection_topic = rospy.get_param("~detection_topic", "detection_result")
		droneposition_topic = rospy.get_param("~droneposition_topic", "drone_position")
		self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
		self.detection_sub = rospy.Subscriber(detection_topic, Detection2DArray, self.detection_cb, queue_size=1)
		self.position_sub = rospy.Subscriber(droneposition_topic, Point, self.position_cb, queue_size=1)
		self.image_center_x = 320
		self.image_center_y = 240
		self.centroid_buffer = []
		self.speed_limit = 10
		self.drone_position = None
		print('NODE STARTED')

	def position_cb(self, data):
		self.drone_position = data
		#print(f"Drone position received - X: {data.x}, Y: {data.y}, Z: {data.z}")

	def detection_cb(self, data):
		
		total_detections = len(data.detections)
		
		if total_detections > 0:
			centroid_x, centroid_y = self.get_centroid(data.detections)
			self.centroid_buffer.append((centroid_x, centroid_y))
		else:
			return

		if self.centroid_buffer:
			avg_centroid_x = sum(x for x, _ in self.centroid_buffer) / len(self.centroid_buffer)
			avg_centroid_y = sum(y for _, y in self.centroid_buffer) / len(self.centroid_buffer)
			print('Centroid Debug - X:', avg_centroid_x, 'Y:', avg_centroid_y)
			self.move_to_centroid(avg_centroid_x, avg_centroid_y)
			if len(self.centroid_buffer) >= 3:
				self.centroid_buffer.pop(0)

	def get_centroid(self,detections):
		centroid_x = 0.0
		centroid_y = 0.0
		players = []
		ball_detected = False

		for detection in detections:
			if detection.results[0].id == 0:
				ball_x = detection.bbox.center.x
				ball_y = detection.bbox.center.y
				ball_detected = True
				break

		for detection in detections:
			if detection.results[0].id == 1:  # Giocatori
				if ball_detected:
					distance = ((detection.bbox.center.x - ball_x) ** 2 +
								(detection.bbox.center.y - ball_y) ** 2) ** 0.5
				else:
					distance = ((detection.bbox.center.x - self.image_center_x) ** 2 +
								(detection.bbox.center.y - self.image_center_y) ** 2) ** 0.5
				players.append({
					'x': detection.bbox.center.x,
					'y': detection.bbox.center.y,
					'distance': distance
				})

		players_sorted = sorted(players, key=lambda player: player['distance'])

		# Considera i 6 giocatori più vicini
		closest_players = players_sorted[:min(6, len(players_sorted))]

		# Calcola il centroide dei giocatori più vicini
		if ball_detected:  # Se la palla è stata rilevata

			# Coordinata x
			weighted_ball_x = ball_x * 0.8
			weighted_players_x = sum(player['y'] for player in closest_players) * 0.2 / len(closest_players)
			centroid_x = weighted_ball_x + weighted_players_x
			
			# Coordinata Y
			weighted_ball_y = ball_y * 0.8
			weighted_players_y = sum(player['y'] for player in closest_players) * 0.2 / len(closest_players)
			centroid_y = weighted_ball_y + weighted_players_y

		else:  # Se la palla non è stata rilevata
			centroid_x = sum(player['x'] for player in closest_players) / len(closest_players)
			centroid_y = sum(player['y'] for player in closest_players) / len(closest_players)

		return centroid_x, centroid_y


	def move_to_centroid(self, centroid_x, centroid_y):
		
		twist_msg = Twist()
		# Calcolo del vettore velocità lineare
		linear_speed_x = 0.05
		linear_speed_y = 0.05

		# Calcolo degli errori rispetto al centro dell'immagine
		error_x = self.image_center_x - centroid_x
		error_y = self.image_center_y - centroid_y

		# Calcolo delle velocità lineari lungo gli assi x e z
		# la X dell'immagine YOLO su unity corrisponde alla Z mentre la Y alla X
		twist_msg.linear.z = linear_speed_x * error_x
		twist_msg.linear.x = linear_speed_y * error_y 

		# Satura la velocità
		if abs(twist_msg.linear.x) > self.speed_limit:
			twist_msg.linear.x = self.speed_limit if twist_msg.linear.x > 0 else -self.speed_limit
		if abs(twist_msg.linear.z) > self.speed_limit:
			twist_msg.linear.z = self.speed_limit if twist_msg.linear.z > 0 else -self.speed_limit

		# Azzera la velocità su x/z quando si trova in prossimità dei bordi (x:-28,0 - z:0, 15)
		if (twist_msg.linear.x < 0 and self.drone_position.x <= -27) or \
			(twist_msg.linear.x > 0 and self.drone_position.x >= -1):
			twist_msg.linear.x = 0
		if (twist_msg.linear.z < 0 and self.drone_position.z <= 1) or \
			(twist_msg.linear.z > 0 and self.drone_position.z >= 14):
			twist_msg.linear.z = 0

        # Pubblica il messaggio di velocità
		print('Debug - Linear X:', twist_msg.linear.x, 'Linear Z:', twist_msg.linear.z) 
		self.vel_pub.publish(twist_msg)



if __name__ == "__main__":
	rospy.init_node("drone_controller")
	node = DroneControllerNode()
	rospy.spin()
