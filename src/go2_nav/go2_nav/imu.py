"""go2_imu: la IMU del CUERPO del Go2 como sensor_msgs/Imu, para GLIM.

  /sportmodestate (unitree_go/SportModeState, ~300-500 Hz, reloj del perro) -> /go2/imu (frame 'imu')

Por que esta IMU y no /utlidar/imu: la IMU interna del L1 esta girada ~122 grados respecto a la nube
(medido con 'go diagnostico'); con ella GLIM/FAST-LIO divergian. La del cuerpo esta alineada con
base_link (URDF: imu_joint en (-0.02557, 0, 0.04232), sin giro) y su sello es el MISMO reloj del
perro que la nube del L1 -> GLIM las sincroniza sin desfase.
Aceleracion en m/s2, giro en rad/s. Cuaternion de Unitree: [w, x, y, z].
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from unitree_go.msg import SportModeState


class ImuCuerpo(Node):
    def __init__(self):
        super().__init__("go2_imu")
        self.pub = self.create_publisher(Imu, "/go2/imu", qos_profile_sensor_data)
        self.create_subscription(SportModeState, "/sportmodestate", self.cb, qos_profile_sensor_data)
        self.ult = (0, 0)
        self.n = 0
        self.create_timer(5.0, self.revisar)
        self.get_logger().info("go2_imu: /sportmodestate -> /go2/imu (IMU del cuerpo, para GLIM)")

    def cb(self, m):
        st = (int(m.stamp.sec), int(m.stamp.nanosec))
        if st <= self.ult:
            return                          # GLIM necesita sellos crecientes
        self.ult = st
        i = Imu()
        i.header.stamp.sec, i.header.stamp.nanosec = st
        i.header.frame_id = "imu"
        q = m.imu_state.quaternion
        i.orientation.w, i.orientation.x, i.orientation.y, i.orientation.z = (float(v) for v in q)
        g, a = m.imu_state.gyroscope, m.imu_state.accelerometer
        i.angular_velocity.x, i.angular_velocity.y, i.angular_velocity.z = (float(v) for v in g)
        i.linear_acceleration.x, i.linear_acceleration.y, i.linear_acceleration.z = (float(v) for v in a)
        self.pub.publish(i)
        self.n += 1

    def revisar(self):
        if self.n == 0:
            self.get_logger().warn("no llega /sportmodestate (cable? perro prendido?)", throttle_duration_sec=15.0)
        self.n = 0


def main():
    rclpy.init()
    n = ImuCuerpo()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
