import sys
import rclpy
from rclpy.node import Node
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from sensor_msgs.msg import Image as ImageMsg
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2

class DroneGUI(Node):
    def __init__(self):
        super().__init__('drone_gui')
        self.bridge = CvBridge()
        self.root = tk.Tk()
        self.root.title("Drone Cameras + World Positions")

        self.drones = ['X3_1', 'X3_2', 'X3_3']
        self.camera_labels = {}
        self.pos_labels = {}
        self.photo_images = {}
        self.odom_subscribers = {}
        self.image_subscribers = {}

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill="both", expand=True)

        for drone_name in self.drones:
            drone_frame = ttk.LabelFrame(main_frame, text=drone_name, padding="5")
            drone_frame.pack(side="left", padx=10, pady=10, fill="both", expand=True)

            camera_label = ttk.Label(drone_frame, text=f"Waiting for /{drone_name}/camera/image_raw")
            camera_label.pack(fill="both", expand=True)
            self.camera_labels[drone_name] = camera_label

            pos_label = ttk.Label(drone_frame, text="Position: (x=0.0, y=0.0)")
            pos_label.pack(pady=5)
            self.pos_labels[drone_name] = pos_label

            self.image_subscribers[drone_name] = self.create_subscription(
                ImageMsg,
                f'/{drone_name}/camera/image_raw',
                lambda msg, name=drone_name: self.image_callback(msg, name),
                10
            )

            self.odom_subscribers[drone_name] = self.create_subscription(
                Odometry,
                f'/{drone_name}/odom',
                lambda msg, name=drone_name: self.odom_callback(msg, name),
                10
            )

        self.get_logger().info('Drone GUI node is ready.')

        self.root.after(100, self.update_gui)

    def image_callback(self, msg, drone_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            cv_image = cv2.resize(cv_image, (320, 240))
            
            img = Image.fromarray(cv_image)
            imgtk = ImageTk.PhotoImage(image=img)

            self.camera_labels[drone_name].config(image=imgtk)
            self.photo_images[drone_name] = imgtk

        except Exception as e:
            self.get_logger().error(f"Error processing image for {drone_name}: {e}")

    def odom_callback(self, msg, drone_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.pos_labels[drone_name].config(text=f"Position: (x={x:.2f}, y={y:.2f})")

    def update_gui(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(10, self.update_gui)

def main(args=None):
    rclpy.init(args=args)
    gui_node = DroneGUI()
    try:
        gui_node.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
