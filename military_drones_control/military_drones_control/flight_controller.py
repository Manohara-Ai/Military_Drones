import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose

class FlightController(Node):
    def __init__(self):
        super().__init__('flight_controller')

        self.declare_parameter('namespace', rclpy.Parameter.Type.STRING)
        self.declare_parameter('initial_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('initial_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('initial_z', rclpy.Parameter.Type.DOUBLE)

        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.current_x = self.get_parameter('initial_x').get_parameter_value().double_value
        self.current_y = self.get_parameter('initial_y').get_parameter_value().double_value
        self.current_z = self.get_parameter('initial_z').get_parameter_value().double_value

        self.cmd_vel_publisher = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, f'{self.namespace}/pose', self.pose_callback, 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.takeoff_done = False
        self.takeoff_duration = 7.0
        self.takeoff_start_time = self.get_clock().now()

        self.move_done = False
        self.move_duration = 15.0
        self.move_start_time = None
        self.speed = 0.5

        self.pause_done = False
        self.pause_duration = 5.0
        self.pause_start_time = None

        self.second_move_done = False
        self.second_move_duration = 17.0
        self.second_move_start_time = None

    def pose_callback(self, msg):
        self.current_x = msg.position.x
        self.current_y = msg.position.y
        self.current_z = msg.position.z

    def control_loop(self):
        msg = Twist()
        now = self.get_clock().now()

        if not self.takeoff_done:
            elapsed_takeoff = (now - self.takeoff_start_time).nanoseconds * 1e-9
            if elapsed_takeoff < self.takeoff_duration:
                msg.linear.z = 0.5
            else:
                self.takeoff_done = True
                self.move_start_time = now
            self.cmd_vel_publisher.publish(msg)
            return

        if not self.move_done:
            elapsed_move = (now - self.move_start_time).nanoseconds * 1e-9
            if elapsed_move < self.move_duration:
                if self.namespace == "X3_1":
                    msg.linear.x = self.speed
                elif self.namespace == "X3_2":
                    msg.linear.y = -self.speed
                elif self.namespace == "X3_3":
                    msg.linear.x = self.speed
            else:
                self.move_done = True
                self.pause_start_time = now
            self.cmd_vel_publisher.publish(msg)
            return

        if not self.pause_done:
            elapsed_pause = (now - self.pause_start_time).nanoseconds * 1e-9
            if elapsed_pause < self.pause_duration:
                msg.linear.x = msg.linear.y = msg.linear.z = 0.0
            else:
                self.pause_done = True
                self.second_move_start_time = now
            self.cmd_vel_publisher.publish(msg)
            return

        if not self.second_move_done:
            elapsed_second_move = (now - self.second_move_start_time).nanoseconds * 1e-9
            if elapsed_second_move < self.second_move_duration:
                if self.namespace == "X3_1":
                    msg.linear.x = self.speed
                    msg.linear.y = -self.speed
                elif self.namespace == "X3_2":
                    msg.linear.y = -self.speed
                elif self.namespace == "X3_3":
                    msg.linear.y = self.speed
            else:
                self.second_move_done = True
                msg.linear.x = msg.linear.y = msg.linear.z = 0.0
        else:
            msg.linear.x = msg.linear.y = msg.linear.z = 0.0

        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    flight_controller = FlightController()
    rclpy.spin(flight_controller)
    flight_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
