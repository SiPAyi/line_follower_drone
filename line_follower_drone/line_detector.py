import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Vector3

import numpy as np
import cv2
import math


from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf_transformations import euler_from_quaternion

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

QOS_PROFILE_DEFAULT = 10

PI = math.pi

RED_COLOR = (0, 0, 255)
BLUE_COLOR = (255, 0, 0)
GREEN_COLOR = (0, 255, 0)

VECTOR_MAGNITUDE_MINIMUM = 2.5


class LineDetector(Node):
	def __init__(self):
		super().__init__('detector')

		# Subscription for camera images.
		self.subscription_camera = self.create_subscription(
			CompressedImage,
			'/iris/downward_camera/image_raw/compressed',
			self.camera_image_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for drone odometry
		self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_callback, qos)

		# Publisher for vector image (for debug purposes).
		self.publisher_vector_image = self.create_publisher(
			CompressedImage,
			"/debug_images/vector_image",
			QOS_PROFILE_DEFAULT)

		# Publisher for velocity
		self.publisher_ = self.create_publisher(Vector3, '/velocity_vector', QOS_PROFILE_DEFAULT)
		

		self.image_height = 0
		self.image_width = 0
		self.lower_image_height = 0
		self.upper_image_height = 0
		
		self.turn = 0.0
		self.yaw = 0.0
		
		#pid related stuff
		self.error = 0.0
		self.prev_error = 0.0
		self.error_sum = 0.0
		self.error_dif = 0.0
		
		self.offset_error = 0.0
		self.offset_prev_error = 0.0
		self.offset_error_sum = 0.0
		self.offset_error_dif = 0.0


	def publish_debug_image(self, publisher, image):
		message = CompressedImage()
		_, encoded_data = cv2.imencode('.jpg', image)
		message.format = "jpeg"
		message.data = encoded_data.tobytes()
		publisher.publish(message)


	def get_vector_angle_in_radians(self, vector):
		if ((vector[0][0] - vector[1][0]) == 0):  # Right angle vector.
			theta = PI / 2
		else:
			slope = (vector[1][1] - vector[0][1]) / (vector[0][0] - vector[1][0])
			theta = math.atan(slope)

		return theta


	def compute_vectors_from_image(self, image, thresh):
		# Draw contours around the objects found in the image.
		contours = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]

		vectors = []
		for i in range(len(contours)):
			coordinates = contours[i][:, 0, :]

			min_y_value = np.min(coordinates[:, 1])
			max_y_value = np.max(coordinates[:, 1])

			# Get coordinates with minimum and maximum values of y-coords respectively.
			min_y_coords = np.array(coordinates[coordinates[:, 1] == min_y_value])
			max_y_coords = np.array(coordinates[coordinates[:, 1] == max_y_value])

			min_y_coord = min_y_coords[0]
			max_y_coord = max_y_coords[0]

			magnitude = np.linalg.norm(min_y_coord - max_y_coord)
			if (magnitude > VECTOR_MAGNITUDE_MINIMUM):
				# Calculate distance from the rover to the middle point of vector.
				rover_point = [self.image_width / 2, self.lower_image_height]
				middle_point = (min_y_coord + max_y_coord) / 2
				distance = np.linalg.norm(middle_point - rover_point)

				angle = self.get_vector_angle_in_radians([min_y_coord, max_y_coord])
				if angle > 0:
					min_y_coord[0] = np.max(min_y_coords[:, 0])
				else:
					max_y_coord[0] = np.max(max_y_coords[:, 0])

				vectors.append([list(min_y_coord), list(max_y_coord)])
				vectors[-1].append(distance)


			## for debugging visually
			# Calculate midpoint of the line
			mid_x = (min_y_coord[0] + max_y_coord[0]) / 2
			mid_y = (min_y_coord[1] + max_y_coord[1]) / 2
			midpoint = (int(mid_x), int(mid_y))
			image_center = (int(image.shape[1]/2), int(image.shape[0]/2))

			# Draw a small circle at midpoint
			cv2.circle(image, midpoint, radius=5, color=(0, 0, 255), thickness=-1)  # red dot
			cv2.circle(image, image_center, radius=5, color=(0, 0, 255), thickness=-1)  # red dot
			
			cv2.line(image, min_y_coord, max_y_coord, BLUE_COLOR, 1)
			cv2.line(image, image_center, midpoint, GREEN_COLOR, 2)
			cv2.line(image, (int(mid_x), int(0)), (int(mid_x), int(image.shape[0])), GREEN_COLOR, 2)

		return vectors, image


	def process_image_for_edge_vectors(self, image):
		self.image_height, self.image_width, color_count = image.shape

		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale image.
		
		# Separate black (color of edges) pixels from the rest by applying threshold.
		threshold_black = 150
		thresh = cv2.threshold(gray, threshold_black, 255, cv2.THRESH_BINARY_INV)[1]
		
		vectors, image = self.compute_vectors_from_image(image, thresh)
		
		# Sort vectors based on distance from rover, as we only want vectors closest to us.
		vectors = sorted(vectors, key=lambda x: x[2])

#		for vector in vectors:
#			cv2.line(thresh, vectors[0][0], vectors[0][1], BLUE_COLOR, 2)
			
		#cv2.imshow("Drawn Image", thresh)
		#cv2.waitKey(1)

		final_vectors = []
		# Pick one vector if they exist.
		if (len(vectors) > 0):
			#cv2.line(image, vectors[-1][0][0], vectors[-1][0][1], GREEN_COLOR, 2)
			final_vectors.append(vectors[0][:2])

		cv2.imshow('Downward Camera View', image)
		cv2.waitKey(1)
		
		self.publish_debug_image(self.publisher_vector_image, image)
		#print("sent the image")

		return final_vectors


	def camera_image_callback(self, message):
		#print("got the image processing it")
		np_arr = np.frombuffer(message.data, np.uint8)
		image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

		vectors = self.process_image_for_edge_vectors(image)

		if (len(vectors) > 0):
			# --- Extract line points ---
			x1 = float(vectors[0][0][0])
			y1 = float(vectors[0][0][1])
			x2 = float(vectors[0][1][0])
			y2 = float(vectors[0][1][1])

			# --- Calculate angle of the line (orientation error) ---
			angle_rad = math.atan2(x2 - x1, y2 - y1)

			# --- PID for line orientation (yaw alignment) ---
			kp_angle = 0.25
			ki_angle = 0.03
			kd_angle = 0.015

			self.error = angle_rad
			self.error_sum += self.error
			self.error_diff = self.error - self.prev_error

			max_integral_angle = 1.0
			self.error_sum = max(-max_integral_angle, min(max_integral_angle, self.error_sum))

			pid_angle = kp_angle * self.error + ki_angle * self.error_sum + kd_angle * self.error_diff
			self.prev_error = self.error

			# --- Calculate horizontal offset from image center ---
			mid_x = (x1 + x2) / 2
			x_offset = (mid_x - (image.shape[1] / 2)) / (image.shape[1] / 2)  # normalized [-1, 1]

			# --- PID for x offset correction ---
			kp_offset = 0.3
			ki_offset = 0.045
			kd_offset = 0.02

			self.offset_error = x_offset
			self.offset_error_sum += self.offset_error
			self.offset_error_diff = self.offset_error - self.offset_prev_error

			max_integral_offset = 1.0
			self.offset_error_sum = max(-max_integral_offset, min(max_integral_offset, self.offset_error_sum))

			pid_offset = kp_offset * self.offset_error + ki_offset * self.offset_error_sum + kd_offset * self.offset_error_diff
			self.offset_prev_error = self.offset_error

			# --- Final yaw adjustment ---
			#pid_angle = 0.0
			self.yaw += -pid_angle + pid_offset  # Both terms affect yaw directionally

			

			#### This keeps yaw in the [-π, π] range.			
			self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi


		self.publish_velocity(1.0, self.yaw)
		return
		
	def publish_velocity(self, speed, turn):
		msg = Vector3()

		msg.x = speed
		msg.y = turn
		msg.z = 0.0
		self.publisher_.publish(msg)
		
	def odom_callback(self, msg):
	
		# msg.q is [w, x, y, z]
		qw, qx, qy, qz = msg.q

		# Reorder for tf: [x, y, z, w]
		q_tf = [qx, qy, qz, qw]
				
		(roll, pitch, yaw) = euler_from_quaternion(q_tf)

		self.yaw = yaw




def main(args=None):
	rclpy.init(args=args)

	line_detector = LineDetector()

	rclpy.spin(line_detector)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_detector.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':

	main()

