import numpy as np
from fcu_command_publisher.state_machine import State


class MISSION(State):
	def __init__(self, state_machine):
		super().__init__(state_machine)
		self.begin_time = None

	def on_enter(self, node):
		self.begin_time = node.now()
		node.body_cmd.velocity.x = node.requested_velocity[0]
		node.body_cmd.velocity.y = node.requested_velocity[1]
		node.body_cmd.velocity.z = node.requested_velocity[2]

	def loop(self, node):
		if (node.now() - self.begin_time).nanoseconds * 1e-9 > node.requested_flight_time:
			self.state_machine.change_to_state(BRAKE, f"Flight time({node.requested_flight_time} seconds) has passed")

	def on_exit(self, node):
		pass


class BRAKE(State):
	def __init__(self, state_machine):
		super().__init__(state_machine)
		self.desired_acceleration = None
		self.desired_velocity = None
		self.request_time = None
		self.retries = 0

	def on_enter(self, node):
		self.retries = 0
		self.desired_velocity = np.linalg.norm(
			[node.uav_state.velocity.x, node.uav_state.velocity.y, node.uav_state.velocity.z])
		self.request_time = node.now()

	def loop(self, node):
		current_velocity = np.array([node.uav_state.velocity.x, node.uav_state.velocity.y, node.uav_state.velocity.z])
		current_velocity_normalized = current_velocity / np.linalg.norm(current_velocity)
		desired_acceleration = -3.0
		self.desired_velocity = np.max([self.desired_velocity + desired_acceleration * node.dt, 0])
		vel_cmd = current_velocity_normalized * self.desired_velocity
		node.body_cmd.velocity.x = vel_cmd[0]
		node.body_cmd.velocity.y = vel_cmd[1]
		node.body_cmd.velocity.z = vel_cmd[2]

		if np.linalg.norm(current_velocity) < 2.0:
			if (node.now() - self.request_time).nanoseconds * 1e-9 > 1.0 and self.retries < 3:
				self.request_time = node.now()
				self.retries += 1
				node.set_uav_mode('Hold')
				node.get_logger().warn(f'Trying to initiate Hold. attempt: {self.retries}')
				if self.retries == 3:
					node.get_logger().error(f"Cannot entering hold!")
			if node.uav_state.fcu_mode == 'Hold':
				self.state_machine.change_to_state(HOLD, "Velocity is bellow 2.0 m/s")

	def on_exit(self, node):
		pass


class HOLD(State):
	def __init__(self, state_machine):
		super().__init__(state_machine)
		self.request_time = None
		self.retries = None

	def on_enter(self, node):
		node.set_uav_mode('Hold')
		self.retries = 0
		self.request_time = node.now()
		node.get_logger().info('Ready for mission!')

	def loop(self, node):
		if node.valid_mission:
			if self.retries == 0 or (
					(node.now() - self.request_time).nanoseconds * 1e-9 > 1.0 and self.retries < 3):
				self.request_time = node.now()
				self.retries += 1
				node.set_uav_mode('Offboard')
				node.get_logger().warn(f'Trying to initiate Offboard. attempt: {self.retries}')
				if self.retries >= 3:
					node.get_logger().error('Cannot entering Offboard!')
					node.valid_mission = False
					self.retries = 0
			if node.uav_state.fcu_mode == 'Offboard':
				self.state_machine.change_to_state(MISSION, "Mission is valid")

	def on_exit(self, node):
		node.valid_mission = False
