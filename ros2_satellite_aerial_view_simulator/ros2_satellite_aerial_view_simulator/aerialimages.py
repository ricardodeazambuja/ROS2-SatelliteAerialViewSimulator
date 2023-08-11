import math

import numpy as np

import cv2

from sensor_msgs.msg import Image as ImageMsg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from tf2_ros import TransformBroadcaster


from aerialviewgenerator.aerialview import AerialView
from aerialviewgenerator.uavphysics import UAVPhysics

class AerialImagesPublisher(Node):
    def __init__(self):
        super().__init__('aerial_images')
        
        self.declare_parameter('rgb_topic', '/carla/flying_sensor/rgb_down/image')
        self.declare_parameter('depth_topic', '/carla/flying_sensor/depth_down/image')
        self.declare_parameter('twist_topic', '/quadctrl/flying_sensor/ctrl_twist_sp')
        self.declare_parameter('delta_t', 0.02)
        self.declare_parameter('output_size', 352)
        self.declare_parameter('baseurl', 'https://wxs.ign.fr/choisirgeoportail/geoportail/wmts?REQUEST=GetTile&SERVICE=WMTS&VERSION=1.0.0&STYLE=normal&TILEMATRIXSET=PM&FORMAT=image/jpeg&LAYER=ORTHOIMAGERY.ORTHOPHOTOS&TILEMATRIX={z}&TILEROW={y}&TILECOL={x}')
        self.declare_parameter('lat', 48.858327718853104)
        self.declare_parameter('lon', 2.294309636169546)
        self.declare_parameter('bearing', 0.0)
        self.declare_parameter('fov', 73.0)
        self.declare_parameter('z_init', 100.0)
        self.declare_parameter('fake_depth_size', (100, 100))
        self.declare_parameter('fake_depth_value', 100)
        self.declare_parameter('parent_frame', "map")
        self.declare_parameter('child_frame', "flying_sensor")
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        twist_topic = self.get_parameter('twist_topic').value
        self.delta_t = self.get_parameter('delta_t').value
        output_size = self.get_parameter('output_size').value
        self.output_img_size = (output_size, output_size)
        baseurl = self.get_parameter('baseurl').value
        lat0 = self.get_parameter('lat').value
        lon0 = self.get_parameter('lon').value
        self.bearing = self.get_parameter('bearing').value
        self.fov = self.get_parameter('fov').value
        z_init = self.get_parameter('z_init').value
        fake_depth_size = self.get_parameter('fake_depth_size').value
        fake_depth_value = self.get_parameter('fake_depth_value').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value


        ctrl_params = {
                      # Position P gains
                      "Px"    : 1.0, "Py"    : 1.0, "Pz"    : 1.0,

                      # Velocity P-D gains
                      "Pxdot" : 5.0, "Dxdot" : 0.5, "Ixdot" : 5.0,

                      "Pydot" : 5.0, "Dydot" : 0.5, "Iydot" : 5.0,

                      "Pzdot" : 4.0, "Dzdot" : 0.5, "Izdot" : 5.0,

                      # Attitude P gains
                      "Pphi"   : 8.0, "Ptheta" : 8.0, "Ppsi"   : 1.5,

                      # Rate P-D gains
                      "Pp" : 1.5, "Dp" : 0.04,

                      "Pq" : 1.5, "Dq" : 0.04,

                      "Pr" : 1.0, "Dr" : 0.1,

                      # Max Velocities (x,y,z) [m/s]
                      "uMax" : 15.0, "vMax" : 15.0, "wMax" : 5.0,

                      "saturateVel_separately" : True,

                      # Max tilt [degrees]
                      "tiltMax" : 50.0,

                      # Max Rate [degrees/s]
                      "pMax" : 200.0, "qMax" : 200.0, "rMax" : 150.0,

                      # Minimum velocity for yaw follow to kick in [m/s]
                      "minTotalVel_YawFollow" : 0.1,

                      "useIntegral" : True    # Include integral gains in linear velocity control
                      }
        
        for k,v in ctrl_params.copy().items():
            self.declare_parameter(k, v)
            ctrl_params[k] = self.get_parameter(k).value
        
        self.avg = AerialView(zoom=20, baseurl=baseurl)
        self.uav = UAVPhysics(z0=z_init, ctrlType="xyz_vel", ctrlParams=ctrl_params)
        self.twist_linear = [0.0,0.0,0.0]
        self.currPos = [0.0,0.0,z_init]
        self.currLatLon = [lat0, lon0]

        self.cv_bridge = CvBridge()
        
        self.rgb_pub = self.create_publisher(ImageMsg, rgb_topic,1)
        self.depth_pub = self.create_publisher(ImageMsg, depth_topic,1)

        self.fake_depth = self.cv_bridge.cv2_to_imgmsg(np.ones(fake_depth_size, dtype="float32")*fake_depth_value, encoding='passthrough')
        self.fake_depth.header.frame_id = self.child_frame

        self.tf_broadcaster = TransformBroadcaster(self)

        self.receive_control_twist = self.create_subscription(
            Twist,
            twist_topic,
            self.receive_control_twist_cb,
            1)

        self.img_timer = self.create_timer(self.delta_t, self.on_img_timer)
 

    def on_img_timer(self):
        nextPos = np.asarray(self.uav.update([0]*3, self.twist_linear, self.delta_t))
        diffPos = nextPos - self.currPos
        self.currPos = nextPos
        bearing = math.degrees(math.atan2(-diffPos[1],diffPos[0]))+90
        distance = np.linalg.norm(diffPos[:2]) # relative distance

        lat, lon = self.avg.getPointAtDistance(self.currLatLon[0], self.currLatLon[1], distance, bearing)
        self.currLatLon = [lat, lon]

        img = self.avg.getAerialImage(self.currLatLon[0], self.currLatLon[1], self.bearing, self.currPos[2], self.fov, output_size=self.output_img_size)

        img_np = np.asarray(img.convert('RGB'))

        img_msg = self.cv_bridge.cv2_to_imgmsg(img_np, encoding='rgb8')
        img_msg.header.frame_id = self.child_frame
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.fake_depth.header.stamp = img_msg.header.stamp
        self.rgb_pub.publish(img_msg)
        self.depth_pub.publish(self.fake_depth)

        self.publish_transform(*self.currPos, parent=self.parent_frame, child=self.child_frame)
        self.get_logger().info(f'Current position: {-self.currPos[1],-self.currPos[0],self.currPos[2]}')
        self.get_logger().info(f'Current lat/lon: {lat, lon}')


    def publish_transform(self, x, y, z, parent="map", child="flying_sensor"):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


    def receive_control_twist_cb(self, twist):
        self.twist_linear = [-twist.linear.y, twist.linear.x, twist.linear.z]


def main():
    rclpy.init()
    aerial_images = AerialImagesPublisher()
    try:
        rclpy.spin(aerial_images)
    except KeyboardInterrupt:
        pass
    finally:
        aerial_images.get_logger().info(f'Shutting aerial_images down...')
        aerial_images.get_logger().error(f'Final lat/lon: {aerial_images.currLatLon} -  altitude: {aerial_images.currPos[2]}')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
