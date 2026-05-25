import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from controller_manager_msgs.srv import SwitchController, SetHardwareComponentState, LoadController, ConfigureController
from lifecycle_msgs.msg import State


CONTROLLERS = ['spot_joint_controller', 'joint_state_broadcaster']
HARDWARE = 'SpotSystem'


class ControlModeSwitcher(Node):
    def __init__(self):
        super().__init__('control_mode_switcher')

        cb = ReentrantCallbackGroup()

        self._switch_controllers = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
            callback_group=cb)
        self._set_hardware_state = self.create_client(
            SetHardwareComponentState, '/controller_manager/set_hardware_component_state',
            callback_group=cb)
        self._load_controller = self.create_client(
            LoadController, '/controller_manager/load_controller',
            callback_group=cb)
        self._configure_controller = self.create_client(
            ConfigureController, '/controller_manager/configure_controller',
            callback_group=cb)
        self._claim_leases = self.create_client(Trigger, '/claim_leases', callback_group=cb)
        self._claim = self.create_client(Trigger, '/claim', callback_group=cb)
        self._power_on = self.create_client(Trigger, '/power_on', callback_group=cb)
        self._stand = self.create_client(Trigger, '/stand', callback_group=cb)

        self.create_service(Trigger, '~/switch_to_low_level', self._to_low_level,
                            callback_group=cb)
        self.create_service(Trigger, '~/switch_to_high_level', self._to_high_level,
                            callback_group=cb)

        self.get_logger().info('Control mode switcher ready.')
        self.get_logger().info('  Low level:  ros2 service call /control_mode_switcher/switch_to_low_level std_srvs/srv/Trigger {}')
        self.get_logger().info('  High level: ros2 service call /control_mode_switcher/switch_to_high_level std_srvs/srv/Trigger {}')

    def _call(self, client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None, 'service not available'
        future = client.call_async(request)
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        if future.result() is None:
            return None, 'service timed out'
        return future.result(), None

    def _set_hardware(self, state_id):
        req = SetHardwareComponentState.Request()
        req.name = HARDWARE
        req.target_state.id = state_id
        result, err = self._call(self._set_hardware_state, req)
        if err:
            return False, err
        if not result.ok:
            return False, f'failed to set hardware state to {state_id}'
        return True, ''

    def _switch_controllers_active(self, activate):
        for name in CONTROLLERS:
            req = SwitchController.Request()
            if activate:
                req.activate_controllers = [name]
            else:
                req.deactivate_controllers = [name]
            req.strictness = SwitchController.Request.BEST_EFFORT
            self._call(self._switch_controllers, req)
        return True, ''

    def _ensure_controllers_loaded(self):
        for name in CONTROLLERS:
            load_req = LoadController.Request()
            load_req.name = name
            self._call(self._load_controller, load_req)  # no-op if already loaded

            cfg_req = ConfigureController.Request()
            cfg_req.name = name
            self._call(self._configure_controller, cfg_req)  # no-op if already configured

    def _to_low_level(self, request, response):
        self.get_logger().info('Switching to low level...')

        self._call(self._claim_leases, Trigger.Request())
        self._call(self._claim, Trigger.Request())
        self._call(self._power_on, Trigger.Request())
        self._call(self._stand, Trigger.Request())

        ok2, err2 = self._set_hardware(State.PRIMARY_STATE_ACTIVE)
        if not ok2:
            response.success = False
            response.message = f'hardware activation failed: {err2}'
            return response

        self._ensure_controllers_loaded()

        ok3, err3 = self._switch_controllers_active(activate=True)
        if not ok3:
            response.success = False
            response.message = f'controller activation failed: {err3}'
            return response

        self.get_logger().info('Now in LOW LEVEL mode.')
        response.success = True
        response.message = 'Switched to low level'
        return response

    def _to_high_level(self, request, response):
        self.get_logger().info('Switching to high level...')

        self._switch_controllers_active(activate=False)
        self._set_hardware(State.PRIMARY_STATE_UNCONFIGURED)

        result, err = self._call(self._claim, Trigger.Request())
        if result is None or not result.success:
            response.success = False
            response.message = f'claim failed: {err or result.message}'
            return response

        self._call(self._power_on, Trigger.Request())
        self._call(self._stand, Trigger.Request())

        self.get_logger().info('Now in HIGH LEVEL mode.')
        response.success = True
        response.message = 'Switched to high level'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControlModeSwitcher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()
