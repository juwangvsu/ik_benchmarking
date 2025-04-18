import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion
import numpy as np
class TaskNode(Node):
    def __init__(self):
        super().__init__("marker")
        self.publisher2_ = self.create_publisher(Marker, "mymarker", 10)
        self.timer = self.create_timer(1., self.timer_callback)
        self.i = 0
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    # publish base_link to markerframe tf for visualization
    def make_transforms(self, transformation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = transformation[0]

        t.transform.translation.x = float(transformation[1])
        t.transform.translation.y = float(transformation[2])
        t.transform.translation.z = float(transformation[3])
        quat = Quaternion()
        xw, xx, xy, xz = quaternion_from_euler(
            float(transformation[4]), float(transformation[5]), float(transformation[6]))
        t.transform.rotation.x = xx #quat[0]
        t.transform.rotation.y = xy #quat[1]
        t.transform.rotation.z = xz #quat[2]
        t.transform.rotation.w = xw #quat[3]

        self.tf_static_broadcaster.sendTransform(t)

    def timer_callback(self):
        msg = String()
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.g = 1.0 
        marker.color.r = 1.0 
        marker.color.a = 1.0
        xyz=np.random.randn(3)/5.0 + np.array([0.568, 0.156, 0.8])
        rpy=[3.14,0,2.5]
        marker.pose.position.x = xyz[0] 
        marker.pose.position.y = xyz[1] #0.156
        marker.pose.position.z = xyz[2] #.4 
        qw, qx, qy, qz = quaternion_from_euler(
            float(rpy[0]), float(rpy[1]), float(rpy[2]))

        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw

        self.publisher2_.publish(marker)
        self.get_logger().info('marker position: "%s"' % marker.pose.position)
        tfdata= [0.0] *7
        tfdata[0]="markerframe"
        tfdata[1]= xyz[0]
        tfdata[2]= xyz[1]
        tfdata[3]= xyz[2]
        tfdata[4]= rpy[0]
        tfdata[5]= rpy[1]
        tfdata[6]= rpy[2]

        self.make_transforms(tfdata)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = TaskNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
