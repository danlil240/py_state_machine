import rclpy
from rclpy.node import Node

from fcu_driver_interfaces.msg import BodyVelocityCommand
from fcu_driver_interfaces.srv import SetFlightMode
from fcu_driver_interfaces.msg import UAVState
from fcu_command_publisher_interfaces.srv import SetMission
from fcu_command_publisher.state_machine import StateMachine
from fcu_command_publisher.states import *
import numpy as np


class FcuCommandPublisher(Node):

	def __init__(self):
		super().__init__('fcu_cmd_publisher')
		self.dt = None
		self.valid_mission = False
		timer_period = 1.0 / 50.0  # seconds
		self.timer = self.create_timer(timer_period, self.main_loop)
		self.body_velocity_cmd_pub = self.create_publisher(BodyVelocityCommand, 'fcu/command/body/velocity', 10)
		self.uav_state_sub = self.create_subscription(UAVState, 'fcu/state', self.uav_state_callback, 10)
		self.set_flight_mode_srv = self.create_client(SetFlightMode, 'fcu/set_flight_mode')
		self.command_body_velocity_srv = self.create_service(
			SetMission, '~/command_body_velocity', self.command_body_velocity_cb)
		self.last_time = self.get_clock().now()
		self.uav_state = UAVState()
		self.body_cmd = BodyVelocityCommand()
		self.requested_velocity = np.array([0.0, 0.0, 0.0])
		self.requested_flight_time = 0.0

		self.state_machine = StateMachine(self)
		self.state_machine.initiate(MISSION, BRAKE, HOLD)
		self.state_machine.change_to_state(HOLD)

	def main_loop(self):
		self.dt = (self.now() - self.last_time).nanoseconds * 1e-9
		self.last_time = self.get_clock().now()
		self.state_machine.loop()
		self.body_cmd.header.stamp = self.get_clock().now().to_msg()
		self.body_velocity_cmd_pub.publish(self.body_cmd)

	def uav_state_callback(self, msg):
		self.uav_state = msg

	def set_uav_mode(self, mode):
		srv = SetFlightMode.Request()
		srv.mode = mode
		response = self.set_flight_mode_srv.call_async(srv)
		return response

	def command_body_velocity_cb(self, request, response):
		self.requested_velocity = np.array([request.command.x, request.command.y, request.command.z])
		self.requested_flight_time = request.time
		if self.state_machine.current_state.name != 'HOLD':
			response.success = False
			response.message = "Not available for mission!"
		elif not np.isfinite(self.requested_velocity.all()):
			response.success = False
			response.message = "Velocity command is not finite!"
		elif self.requested_flight_time <= 0:
			response.success = False
			response.message = "Flight time not set!"
		else:
			self.valid_mission = True
			self.get_logger().info(
				f'Starting mission at velocity: {self.requested_velocity} for {self.requested_flight_time} seconds!')
			response.success = True
			response.message = "Entering to mission!"
		return response

	def now(self):
		return self.get_clock().now()


def main(args=None):
	rclpy.init(args=args)

	fcu_command_publisher = FcuCommandPublisher()

	rclpy.spin(fcu_command_publisher)

	fcu_command_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
