import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from scipy.interpolate import UnivariateSpline, InterpolatedUnivariateSpline
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

from kartech_linear_actuator_interface_msgs.msg import BrakeControl, BrakePositionReport, KdFreqDeadbandRequest, Heartbeat, LeftJoystick, RightJoystick
from raptor_dbw_msgs.msg import AkitBrakerequest, BrakePressureReport, DbwBrakereport
from analogx_interface_msgs.msg import Analogx1, BrakeSensorBodyTemp, BrakeTemp

from sklearn.preprocessing import MinMaxScaler
from scipy import stats
import pwlf

# TODO: check if the dependencies are visible (analogx, kartech)- shoot out appropriate error message
# TODO: get the variance of the pressure sensor.
# TODO: get covariance for each pressure prediction

BRAKE_MIN = 550
BRAKE_MAX = 2000
RAPTOR_BRAKE_REQ_MIN = 550
RAPTOR_BRAKE_REQ_MAX = 2000

# BRAKE_PRESSURE_MV = np.array([440.0, 535.0, 540.0, 570.0, 620.0, 760.0, 900.0, 1050.0, 1260.0, 1460.0, 1640.0, 1920.0])
# BRAKE_PRESSURE_KPA = (0.5672 * BRAKE_PRESSURE_MV - 246.6) * 6.89476
# BRAKE_POSITIONS_MINCH = np.array([0.925, 1.85, 1.9, 1.95, 2.0, 2.05, 2.1, 2.15, 2.2, 2.25, 2.3, 2.35]) * 1000.0

# Max Stroke: 3.125"
# BRAKE_MAX_POSITION = (3.125 - 0.5) * 1000.0
# BRAKE_MIN_POSITION = (0.55 - 0.5) * 1000.0
BRAKE_MAX_POSITION = 2000
BRAKE_MIN_POSITION = 550

# BRAKE_THRESHOLD = 0.98 * BRAKE_MAX_POSITION

X_POSITION = []
Y_PRESSURE = []
X_VELOCITY = []

# Max Stroke: 3.125"

# BRAKE_POSITIONS_MINCH_CALIB = np.array([1.8, 1.9, 2.0, 2.1, 2.3, 2.35, 2.4, 2.5, 2.6]) * 1000.0
# BRAKE_PRESSURE_KPA_CALIB = np.zeros(len(BRAKE_POSITIONS_MINCH_CALIB))

# below 0.066: steady state error
# above 0.066: oscilation
K = 0.02

# DEADBAND = 5

# deadband command
# ros2 topic pub /kartech_linear_actuator_interface_interface/kd_freq_deadband_request kartech_linear_actuator_interface_msgs/msg/KdFreqDeadbandRequest 
#    '{messagetype: 245, datatype: 1, autoreplyflag: 1, confirmationflag: 0, kd: 65535, errordeadband: 5, byte3: 1}'

# position command
# ros2 topic pub /kartech_linear_actuator_interface_interface/brake_control  kartech_linear_actuator_interface_msgs/msg/BrakeControl 
#    '{position_command: 15, datatype: 10, autoreply_flag: 1, confirmation_flag: 0, dpos_low: 196, dpos_hi: 201, motor_enable: 1, clutch_enable: 1}'

class KartechLinearActuatorController(Node):

    def __init__(self):
        super().__init__('kartech_linear_actuator_controller')
        self.brake_control_publisher_ = self.create_publisher(
            AkitBrakerequest, 
            '/raptor_dbw_interface/akit_brakerequest', 
            qos_profile_sensor_data)
        self.brake_position_report_subscriber_ = self.create_subscription(
            DbwBrakereport,
            '/raptor_dbw_interface/dbw_brakereport',
            self.brake_position_report_callback,
            qos_profile_sensor_data)
        self.brake_pressure_subscriber_ = self.create_subscription(
            BrakePressureReport,
            '/raptor_dbw_interface/brake_pressure_report',
            self.brake_pressure_sensor_callback,
            qos_profile_sensor_data)
        self.analogx_brake_pressure_subscriber_ = self.create_subscription(
            Analogx1,
            '/analogx_interface_interface/analogx1',
            self.analogx_sensor_callback,
            qos_profile_sensor_data)

        self.brake_control_timer = self.create_timer(0.01, self.brake_control_callback)
        self.calibration_timer = self.create_timer(0.05, self.on_calibration_timer)

        self.brake_pressure_kpa = None
        self.analogx_brake_pressure_mv = None
        self.analogx_brake_pressure_kpa = None
        
        self.brake_request = None
        self.calibration_brake_position = None
        self.calibration_start_time = 0
        # self.previous_shaftextension = 0
        self.previous_dbw_brakepdlposnfdbck = 0

        self.brakeModel = BrakeModel()

    def brake_request_callback(self, brake_request):
        self.brake_request = brake_request

    def brake_pressure_sensor_callback(self, brake_pressure_sensor):
        brake_pressure_mv = brake_pressure_sensor.brake_pressure_fdbk_rear
        self.brake_pressure_kpa = (0.5672 * brake_pressure_mv - 246.6) * 6.89476

    def analogx_sensor_callback(self, analogx_sensor):
        self.analogx_brake_pressure_mv = analogx_sensor.brake_pressure
        self.analogx_brake_pressure_kpa = (0.5672 * self.analogx_brake_pressure_mv - 246.6) * 6.89476
    
    def brake_position_report_callback(self, brake_position_report):
        self.brake_position = brake_position_report


    def brake_control_callback(self):
        if not self.brakeModel.calibrated and self.calibration_brake_position is not None:
            # Calibration Mode
            self.send_brake_position(self.calibration_brake_position)
            return
        if self.brake_request is None:
            return
        brake_req_kpa = self.brake_request.akit_brakepedalreq
        position = self.linear_map(brake_req_kpa, RAPTOR_BRAKE_REQ_MIN, RAPTOR_BRAKE_REQ_MAX, BRAKE_MIN, BRAKE_MAX)
        # fdfw_position = self.kpa_to_minch_lookup(brake_req_kpa)
        # fdfw_position = np.interp(brake_req_kpa, BRAKE_PRESSURE_KPA_CALIB, BRAKE_POSITIONS_MINCH_CALIB)
        # fdfw_position = self.brakeModel.pressureToPosition(brake_req_kpa)
        # fdbk_position = 0.0
        # if self.analogx_brake_pressure_kpa is not None:
        #     fdbk_position = (brake_req_kpa - self.analogx_brake_pressure_kpa) * K 
        #     print(f"feedback position: {fdbk_position}")
        # else:
        #     fdbk_position = 0.0
        # position = fdfw_position + fdbk_position
        print(f"kpa: {self.brake_pressure_kpa}, kpa: {self.analogx_brake_pressure_kpa}, brake_request_kpa: {brake_req_kpa}, position: {position}")
        self.send_brake_position(position)
    
    def send_brake_position(self, position):
        msg = AkitBrakerequest()
        msg.stamp = self.get_clock().now().to_msg()
        msg.akit_brakepedalreq = position
        self.brake_control_publisher_.publish(msg)
    
    def linear_map(self, x, x_min, x_max, y_min, y_max):
        return (x - x_min) * (y_max - y_min) / (x_max - x_min) + y_min
    
    def on_calibration_timer(self):
        """ 
        Brake calibration uses a sine wave actuation with a period of 10 seconds that collects
        data on the relationship of brake position and pressure and fits a regressional piecewise 
        function.
        """
        T = 10.0
        stamp = self.get_clock().now().to_msg()
        if self.calibration_brake_position is None:
            self.calibration_start_time = stamp.sec + (stamp.nanosec / 1e9)
        t = stamp.sec + (stamp.nanosec / 1e9) - self.calibration_start_time
        A = -1 * (BRAKE_MAX_POSITION - BRAKE_MIN_POSITION) / 2.0
        D = (BRAKE_MAX_POSITION + BRAKE_MIN_POSITION) / 2.0
        w = (2 * np.pi) / T
        phi = 1 * np.pi / 2.0
        self.calibration_brake_position = A * np.sin( w * t + phi) + D

        if self.brake_position.dbw_brakepdlposnfdbck is not None and self.analogx_brake_pressure_kpa is not None:
            if self.brake_position.dbw_brakepdlposnfdbck < 500.0:
                return
            X_POSITION.append(self.brake_position.dbw_brakepdlposnfdbck)
            Y_PRESSURE.append(self.analogx_brake_pressure_kpa)
            X_VELOCITY.append(self.brake_position.dbw_brakepdlposnfdbck - self.previous_dbw_brakepdlposnfdbck)
            print(f"[{t:.2f}] request: {self.calibration_brake_position:.2f}, position: {self.brake_position.dbw_brakepdlposnfdbck}, pressure: {self.analogx_brake_pressure_kpa}, velocity: {self.brake_position.dbw_brakepdlposnfdbck - self.previous_dbw_brakepdlposnfdbck}")
            self.previous_dbw_brakepdlposnfdbck = self.brake_position.dbw_brakepdlposnfdbck

        if len(X_POSITION) >  T / (self.calibration_timer.timer_period_ns / 1e9) / 2:
            positions = X_POSITION
            pressures = Y_PRESSURE
            velocities = X_VELOCITY
            self.brakeModel.calibrate(positions, pressures, velocities, enable_visualization=True)
            self.calibration_timer.cancel()

class BrakeModel:
    def __init__(self):
        self.models = {"sigmoid": 0, "piecewise linear curve": 1}
        self._model = self.models["piecewise linear curve"]
        self._calibrated = False
        self.scale_factor = 1000000.0
        self.min_position = None
        self.max_position = None
        self.min_pressure = None
        self.max_pressure = None
        self.a = None
        self.b = None
        self.c = None
        self.d = None
        self.e = None
        self.f = None
        self.piecewise_linear_curve = None
        self.inverse_piecewise_linear_curve = None

    def pressureToPosition(self, pressure):
        if not self.calibrated:
            raise Exception("BrakeModel must be calibrated before interpolation.")
        if pressure < self.min_pressure:
            return self.min_position
        if pressure > self.max_pressure:
            return self.max_position
        if self._model == self.models["sigmoid"]:
            position = self.logit(pressure / self.scale_factor, self.a, self.b, self.c, self.d, self.e, self.f)
            position = position * self.scale_factor
        elif self._model == self.models["piecewise linear curve"]:
            position = self.inverse_piecewise_linear_curve.predict(pressure)
        return position
    
    def calibrate(self, positions, pressures, velocities, enable_visualization=False):
        self.min_position = min(positions)
        self.max_position = max(positions)
        self.min_pressure = min(pressures)
        self.max_pressure = max(pressures)
        # TOOD: record velocity differential
        if self._model == self.models["sigmoid"]:
            self.calibrateSigmoid(positions, pressures, enable_visualization)
        else:
            self.calibratePieceWiseLinearCurve(positions, pressures, enable_visualization)

    def calibratePieceWiseLinearCurve(self, positions, pressures, enable_visualization=False):
        X = np.array(positions)
        Y = np.array(pressures)
        self.piece_wise_linear_curve = pwlf.PiecewiseLinFit(X, Y)
        self.piece_wise_linear_curve.fit(4)
        self.inverse_piecewise_linear_curve = pwlf.PiecewiseLinFit(self.piece_wise_linear_curve.predict(self.piece_wise_linear_curve.fit_breaks), self.piece_wise_linear_curve.fit_breaks)
        self.inverse_piecewise_linear_curve.fit_with_breaks(self.piece_wise_linear_curve.predict(self.piece_wise_linear_curve.fit_breaks))
        if enable_visualization:
            plt.figure(1)
            u = np.linspace(min(X), max(X), num=10000)
            y_fit = self.piece_wise_linear_curve.predict(u)
            plt.scatter(X, Y, s=10)
            plt.plot(u, y_fit, color='red', linewidth=2)
            plt.xlabel("Position")
            plt.ylabel("Pressure")
            plt.title("Piecewise Linear Curve")

            plt.figure(2)
            u = np.linspace(min(self.piece_wise_linear_curve.predict(self.piece_wise_linear_curve.fit_breaks)), max(self.piece_wise_linear_curve.predict(self.piece_wise_linear_curve.fit_breaks)), num=10000)
            y_fit = self.inverse_piecewise_linear_curve.predict(u)
            plt.scatter(Y, X, s=10)
            plt.scatter(self.piece_wise_linear_curve.predict(self.piece_wise_linear_curve.fit_breaks), self.piece_wise_linear_curve.fit_breaks, color="green")
            plt.plot(u, y_fit, color='red', linewidth=2)
            plt.xlabel("Pressure")
            plt.ylabel("Position")
            plt.title("Inverse Piecewise Linear Curve")
            plt.show()
            # plt.show(block=False)
        self.calibrated = True

    def calibrateSigmoid(self, positions, pressures, enable_visualization=False):
        X = np.array(positions) / self.scale_factor
        Y = np.array(pressures) / self.scale_factor
        p0 = [max(Y), 1, 1, 1, np.median(X), min(Y)]
        popt, pcov = curve_fit(self.sigmoid, X, Y, p0, maxfev=50000)
        self.a, self.b, self.c, self.d, self.e, self.f = popt

        if enable_visualization:
            # Visualize the regressional sigmoid function. Note this will block the main thread.
            plt.figure(1)
            u = np.linspace(np.min(X), np.max(X), 100)
            y_fit = self.sigmoid(u, self.a, self.b, self.c, self.d, self.e, self.f)
            residuals = Y - self.sigmoid(Y, *popt)
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((Y - np.mean(Y))**2)
            r_squared = 1 - (ss_res / ss_tot)
            plt.scatter(X * self.scale_factor, Y* self.scale_factor, s=10)
            plt.plot(u * self.scale_factor, y_fit * self.scale_factor, color='red',linewidth=2, label=f"r^2 = {r_squared}")
            plt.xlabel("Position")
            plt.ylabel("Pressure")
            plt.title("Sigmoid Regression")
            print(f"{self.a} / ({self.b} + {self.c} * np.exp(-{self.d} * (x - {self.e}))) + {self.f}")

            plt.figure(2)
            min_y = self.sigmoid(np.min(X), self.a, self.b, self.c, self.d, self.e, self.f)
            max_y = self.sigmoid(np.max(X), self.a, self.b, self.c, self.d, self.e, self.f)
            print(f"min_x: {np.min(X) * self.scale_factor}, max_x: {np.max(X) * self.scale_factor}")
            print(f"min predicted pressure: {min_y * self.scale_factor}, max predicted pressure: {max_y * self.scale_factor}")
            undefined_pressure = (self.a / self.b) + (self.f / self.c) * self.scale_factor
            print(f"undefined: pressure < {undefined_pressure}")
            u = np.linspace(min_y, max_y, 100)

            for i in u:
                print(i)
                y_fit = self.logit(i, self.a, self.b, self.c, self.d, self.e, self.f)
                print(y_fit)
            y_fit = self.logit(u, self.a, self.b, self.c, self.d, self.e, self.f)
            residuals = X - self.logit(X, *popt)
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((X - np.mean(X))**2)
            r_squared = 1 - (ss_res / ss_tot)
            plt.scatter(Y * self.scale_factor, X* self.scale_factor, s=10)
            plt.plot(u * self.scale_factor, y_fit * self.scale_factor, color='red',linewidth=2, label=f"r^2 = {r_squared}")
            plt.xlabel("Pressure")
            plt.ylabel("Position")
            plt.title("Logistic Regression")
            print(f"log({self.a} / ({self.c} * (x - {self.f}))) / (-{self.d}) + {self.e}")
            plt.show()
        self.calibrated = True
    
    def calibrateCubeRoot(self, positions, pressures, enable_visualization=False):
        X = np.array(positions)
        Y = np.array(pressures)
        popt, pcov = curve_fit(self.cubic_root, X, Y, maxfev=50000)
        self.a, self.b, self.c = popt

        if enable_visualization:
            plt.figure(1)
            u = np.linspace(np.min(X), np.max(X), 100)
            y_fit = self.cubic_root(u, self.a, self.b, self.c)
            residuals = Y - self.cubic_root(Y, *popt)
            ss_res = np.sum(residuals**2)
            ss_tot = np.sum((Y - np.mean(Y))**2)
            r_squared = 1 - (ss_res / ss_tot)
            plt.scatter(X, Y, s=10)
            plt.plot(u, y_fit, color='red',linewidth=2, label=f"r^2 = {r_squared}")
            plt.xlabel("Position")
            plt.ylabel("Pressure")
            plt.title("Cube Root Regression")
            plt.show()
    
    def cubic_root(self, x, a, b, c):
        return a * pow(x - b, 1/3) + c

    def sigmoid(self, x, a, b, c, d, e, f):
        return a / (b + c * np.exp(-d * (x - e))) + f
    
    def logit(self, x, a, b, c, d, e, f):
        """ The inverse of a sigmoid function is a logit function. """
        return np.log(a / (c * x - f) - b / c) / (-d) + e

    @property
    def model(self):
        return self._model

    @model.setter
    def model(self, value):
        if value not in self.models:
            raise ValueError(f"BrakeModel: '{model}' does not exist.")
        self._model = value

    @property
    def calibrated(self):
        return self._calibrated

    @calibrated.setter
    def calibrated(self, value):
        self._calibrated = value


def main(args=None):
    rclpy.init(args=args)
    kartech_linear_actuator_controller = KartechLinearActuatorController()
    rclpy.spin(kartech_linear_actuator_controller)
    kartech_linear_actuator_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
