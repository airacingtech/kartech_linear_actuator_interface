import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from scipy.interpolate import UnivariateSpline, InterpolatedUnivariateSpline
import numpy as np
import matplotlib.pyplot as plt

from kartech_linear_actuator_interface_msgs.msg import BrakeControl, BrakePositionReport, KdFreqDeadbandRequest, Heartbeat, LeftJoystick, RightJoystick
from raptor_dbw_msgs.msg import AkitBrakerequest, BrakePressureReport
from analogx_interface_msgs.msg import Analogx1, BrakeSensorBodyTemp, BrakeTemp


BRAKE_MIN = 550
BRAKE_MAX = 2000
RAPTOR_BRAKE_REQ_MIN = 0
RAPTOR_BRAKE_REQ_MAX = 2000

BRAKE_PRESSURE_MV = np.array([440.0, 535.0, 540.0, 570.0, 620.0, 760.0, 900.0, 1050.0, 1260.0, 1460.0, 1640.0, 1920.0])
BRAKE_PRESSURE_KPA = (0.5672 * BRAKE_PRESSURE_MV - 246.6) * 6.89476
BRAKE_POSITIONS_MINCH = np.array([0.925, 1.85, 1.9, 1.95, 2.0, 2.05, 2.1, 2.15, 2.2, 2.25, 2.3, 2.35]) * 1000.0

# Max Stroke: 3.125"
# wave through entire range, collect pressure data
#   - possiblly check when the brake position report has maxed out in n trials and cut off the range test.
# fit cubic spline
# cut off the position domain with respect to a pressure range.
# do this dynamically during callibration.
# remove human in the loop to change the input position values for interpolation.

BRAKE_POSITIONS_MINCH_CALIB = np.array([0.7, 1.0, 1.6, 1.9, 2.0, 2.15]) * 1000.0#, 2.7, 2.8, 2.9, 3.0]) * 1000.0
BRAKE_PRESSURE_KPA_CALIB = np.zeros(len(BRAKE_POSITIONS_MINCH_CALIB))

# below 0.066: steady state error
# above 0.066: oscilation
K = 0.01

DEADBAND = 5

MAX_POSITION = np.max(BRAKE_POSITIONS_MINCH_CALIB)

# deadband command
# ros2 topic pub /kartech_linear_actuator_interface_interface/kd_freq_deadband_request kartech_linear_actuator_interface_msgs/msg/KdFreqDeadbandRequest 
#    '{messagetype: 245, datatype: 1, autoreplyflag: 1, confirmationflag: 0, kd: 65535, errordeadband: 5, byte3: 1}'

# position command
# ros2 topic pub /kartech_linear_actuator_interface_interface/brake_control  kartech_linear_actuator_interface_msgs/msg/BrakeControl 
#    '{position_command: 15, datatype: 10, autoreply_flag: 1, confirmation_flag: 0, dpos_low: 196, dpos_hi: 201, motor_enable: 1, clutch_enable: 1}'

class KartechLinearActuatorController(Node):

    def __init__(self):
        super().__init__('kartech_linear_actuator_controller')
        self.brake_control_publisher_ = self.create_publisher(BrakeControl, '/kartech_linear_actuator_interface_interface/brake_control', qos_profile_sensor_data)
        self.brake_deadband_publisher_ = self.create_publisher(KdFreqDeadbandRequest, '/kartech_linear_actuator_interface_interface/kd_freq_deadband_request', qos_profile_sensor_data)
        self.brake_request_subscriber_ = self.create_subscription(
            AkitBrakerequest,
            '/raptor_dbw_interface/akit_brakerequest',
            self.brake_request_callback,
            qos_profile_sensor_data)
        self.brake_pressure_subscriber_ = self.create_subscription(
            BrakePressureReport,
            '/raptor_dbw_interface/brake_pressure_report',
            self.brake_pressure_sensor_callback,
            qos_profile_sensor_data)
        self.brake_position_report_subscriber_ = self.create_subscription(
            BrakePositionReport,
            '/kartech_linear_actuator_interface_interface/brake_position_report',
            self.brake_position_report_callback,
            qos_profile_sensor_data)
        self.analogx_brake_pressure_subscriber_ = self.create_subscription(
            Analogx1,
            '/analogx_interface_interface/analogx1',
            self.analogx_sensor_callback,
            qos_profile_sensor_data)
        self.analogx_brake_pressure_subscriber_ = self.create_subscription(
            Analogx1,
            '/analogx_interface_interface/analogx1',
            self.analogx_sensor_callback,
            qos_profile_sensor_data)
        self.estop_subscriber = self.create_subscription(
            Heartbeat,
            '/kartech_linear_actuator_interface_interface/heartbeat',
            self.estop_callback,
            qos_profile_sensor_data)

        self.brake_control_timer = self.create_timer(0.01, self.brake_control_callback)
        self.calibration_timer = self.create_timer(3.0, self.on_calibration_timer)

        self.brake_pressure_kpa = None
        self.analogx_brake_pressure_mv = None
        self.analogx_brake_pressure_kpa = None
        self.brake_request = None
        self.estop_override = False
        self.calibrated = False
        self.calibration_read_idx = -1
        self.calibration_send_idx = 0
        self.callibration_brake_position = None

        self.send_deadband(DEADBAND)
    
    def estop_callback(self, estop_request):
        self.estop_override = bool(estop_request.e_stop_indication)

    def brake_request_callback(self, brake_request):
        self.brake_request = brake_request

    def brake_control_callback(self):
        if self.estop_override:
            # E-Stop
            self.send_brake_position(MAX_POSITION)
            return
        if not self.calibrated and self.callibration_brake_position is not None:
            # Calibration Mode
            self.send_brake_position(self.callibration_brake_position)
            return
        if not self.calibrated:
            return
        if self.brake_request is None:
            return
        if self.brake_request.akit_brakepedalreq == 0:
            self.send_brake_position(500)
            return
        brake_req_kpa = self.brake_request.akit_brakepedalreq
        # position = self.linear_map(brake_request.akit_brakepedalreq, RAPTOR_BRAKE_REQ_MIN, RAPTOR_BRAKE_REQ_MAX, BRAKE_MIN, BRAKE_MAX)
        # fdfw_position = self.kpa_to_minch_lookup(brake_req_kpa)
        fdfw_position = np.interp(brake_req_kpa, BRAKE_PRESSURE_KPA_CALIB, BRAKE_POSITIONS_MINCH_CALIB)
        fdbk_position = 0.0
        if self.analogx_brake_pressure_mv is not None:
            fdbk_position = (brake_req_kpa - self.analogx_brake_pressure_kpa) * K # why is this diff kpa - mv?
        else:
            fdbk_position = 0.0
        position = fdfw_position + fdbk_position
        print(self.analogx_brake_pressure_kpa, brake_req_kpa, position)
        self.send_brake_position(position)

    def brake_pressure_sensor_callback(self, brake_pressure_sensor):
        brake_pressure_mv = brake_pressure_sensor.brake_pressure_fdbk_front
        self.brake_pressure_kpa = (0.5672 * brake_pressure_mv - 246.6) * 6.89476
    
    def brake_position_report_callback(self, brake_position_report):
        self.brake_position = brake_position_report

    def analogx_sensor_callback(self, analogx_sensor):
        self.analogx_brake_pressure_mv = analogx_sensor.brake_pressure
        self.analogx_brake_pressure_kpa = (0.5672 * self.analogx_brake_pressure_mv - 246.6) * 6.89476
    
    def send_deadband(self, deadband):
        msg = KdFreqDeadbandRequest()
        msg.stamp = self.get_clock().now().to_msg()
        msg.messagetype = 245
        msg.datatype = 1
        msg.autoreplyflag = 1
        msg.confirmationflag = 0
        msg.kd = 65535
        msg.errordeadband = deadband
        msg.byte3 = 1
        self.brake_deadband_publisher_.publish(msg)
        
    def send_brake_position(self, position):
        position += 500.0
        msg = BrakeControl()
        msg.stamp = self.get_clock().now().to_msg()
        dpos_low, dpos_hi = self.split_brake_position(int(position))
        motor_enable = 1 if self.brake_position.shaftextension - 500.0 < MAX_POSITION or position - 500.0 < MAX_POSITION else 0
        msg.position_command = 15
        msg.datatype = 10
        msg.autoreply_flag = 1
        msg.confirmation_flag = 0
        msg.dpos_low = dpos_low
        msg.dpos_hi = dpos_hi
        msg.motor_enable = motor_enable
        msg.clutch_enable = 1
        self.brake_control_publisher_.publish(msg)
    
    def split_brake_position(self, position):
        dpos_low = position & 0x00FF
        dpos_hi = (position >> 8) & 0x001F
        return dpos_low, dpos_hi
    
    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min
    
    def on_calibration_timer(self):
        enable_visualization = False
        if self.calibration_read_idx >= len(BRAKE_PRESSURE_KPA_CALIB) and self.calibration_send_idx >= len(BRAKE_POSITIONS_MINCH_CALIB):
            print(BRAKE_PRESSURE_KPA_CALIB)
            print(BRAKE_POSITIONS_MINCH_CALIB)
            try:
                self.kpa_to_minch_lookup = InterpolatedUnivariateSpline(BRAKE_PRESSURE_KPA_CALIB, BRAKE_POSITIONS_MINCH_CALIB)
            except ValueError as e:
                print(e)
                print("Retrying calibration...")
                self.calibration_read_idx = -1
                self.calibration_send_idx = 0
                self.callibration_brake_position = None
                return
            self.calibrated = True
            MAX_POSITION = np.max(BRAKE_POSITIONS_MINCH_CALIB)
            self.calibration_timer.cancel()
            if enable_visualization:    
                # Visualize cubic spline interpolation. Note this will block the main thread.
                test = np.linspace(460.0, 3000.0, 100)
                plt.plot(test, self.kpa_to_minch_lookup(test))
                plt.scatter(BRAKE_PRESSURE_KPA_CALIB, BRAKE_POSITIONS_MINCH_CALIB)
                plt.show() 
            return
        if self.calibration_read_idx >= 0:
            BRAKE_POSITIONS_MINCH_CALIB[self.calibration_read_idx] = self.brake_position.shaftextension - 500.0
            BRAKE_PRESSURE_KPA_CALIB[self.calibration_read_idx] = self.analogx_brake_pressure_kpa
        if self.calibration_send_idx < len(BRAKE_POSITIONS_MINCH_CALIB):
            self.callibration_brake_position = BRAKE_POSITIONS_MINCH_CALIB[self.calibration_send_idx]
        self.calibration_read_idx += 1
        self.calibration_send_idx += 1
        
def main(args=None):
    rclpy.init(args=args)
    kartech_linear_actuator_controller = KartechLinearActuatorController()
    rclpy.spin(kartech_linear_actuator_controller)
    kartech_linear_actuator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
