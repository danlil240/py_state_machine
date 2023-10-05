class StateMachine:
	def __init__(self, node):
		self.current_state = None
		self.states = {}
		self.node = node

	def register_state(self, state_class):
		self.states[state_class.__name__] = state_class(self)
		self.states[state_class.__name__].name = state_class.__name__

	def initiate(self, *args):
		for cls in args:
			self.register_state(cls)

	def change_to_state(self, new_state, reason=""):
		if self.current_state is not None:
			self.node.get_logger().info(
				f"State change: |{self.current_state.name}| ==> |{new_state.__name__}|, Reason: {reason}")
			self.current_state.on_exit(self.node)
		self.current_state = self.states[new_state.__name__]
		self.current_state.on_enter(self.node)

	def loop(self):
		self.current_state.loop(self.node)


class State(object):
	def __init__(self, state_machine):
		self.state_machine = state_machine

	def on_enter(self, node):
		pass

	def loop(self, node):
		pass

	def on_exit(self, node):
		node.get_logger().info(f"Exiting [{self.__name__}]")
