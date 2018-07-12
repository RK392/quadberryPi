import sys, getopt

sys.path.append('.')
import RTIMU
import time
import math
import logging

SETTINGS_FILE = "RTIMULib"

class IMU:
    def __init__(self):
        self._settings = RTIMU.Settings(SETTINGS_FILE)
        self._imu = RTIMU.RTIMU(self._settings)

        self._data = None

        # offsets
        self.yawoff = 0.0
        self.pitchoff = 0.0
        self.rolloff = 0.0

        # timers
        self.t_print = time.time()
        self.t_damp = time.time()
        self.t_fail = time.time()
        self.t_fail_timer = 0.0
        self.t_shutdown = 0

        self.poll_interval = 0

        self.print_interval = 0.1

        self.magnetic_deviation = 0

        # magnetic deviation
        f = open('mag', 'r')
        self.magnetic_deviation = float(f.readline())
        f.close()

        # data variables
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.heading = 0.0
        self.rollrate = 0.0
        self.pitchrate = 0.0
        self.yawrate = 0.0

        # dampening variables
        self.t_one = 0
        self.t_three = 0
        self.roll_total = 0.0
        self.roll_run = [0] * 10
        self.heading_cos_total = 0.0
        self.heading_sin_total = 0.0
        self.heading_cos_run = [0] * 30
        self.heading_sin_run = [0] * 30

        # callback functions
        self.callbacks = []

        self.stop_signal = 0

        self.hack = time.time()

    def initialize(self):
        if (not self._imu.IMUInit()):
            self.hack = time.time()
            if (self.hack - self.t_print) > 1.0:
                logging.info("IMU Failed to initialize! Retrying...")
                self.t_shutdown += 1
                if self.t_shutdown > 9:
                    raise Exception("IMU Failed to initialize! Max retries exceeded!")

        self._imu.setSlerpPower(0.02)
        self._imu.setGyroEnable(True)
        self._imu.setAccelEnable(True)
        self._imu.setCompassEnable(True)

        self.poll_interval = self._imu.IMUGetPollInterval()

    def get_imu_reading(self):
        return self.heading, self.roll, self.pitch

    def register_callback(self, data_callback):
        self.callbacks.append(data_callback)

    def stop(self):
        self.stop_signal = 1

    def run(self):
        self.stop_signal = 0
        while self.stop_signal == 0:
            self.hack = time.time()

            # if it's been longer than 5 seconds since last print
            if (self.hack - self.t_damp) > 5.0:

                if (self.hack - self.t_fail) > 1.0:
                    self.t_one = 0
                    self.t_three = 0
                    self.roll_total = 0.0
                    self.roll_run = [0] * 10
                    self.heading_cos_total = 0.0
                    self.heading_sin_total = 0.0
                    self.heading_cos_run = [0] * 30
                    self.heading_sin_run = [0] * 30
                    self.t_fail_timer += 1
                    self.t_fail = self.hack
                    self.t_shutdown += 1
                    logging.debug("IMU Failed! Retrying...")

            if self._imu.IMURead():
                self._data = self._imu.getIMUData()
                #logging.debug('IMU Data: ' + str(self._data))
                fusionPose = self._data["fusionPose"]
                gyro = self._data["gyro"]
                self.t_fail_timer = 0.0

                if (self.hack - self.t_damp) > .1:
                    self.roll = round(math.degrees(fusionPose[0]) - self.rolloff, 1)
                    self.pitch = round(math.degrees(fusionPose[1]) - self.pitchoff, 1)
                    self.yaw = round(math.degrees(fusionPose[2])- self.yawoff, 1)
                    self.rollrate = round(math.degrees(gyro[0]), 1)
                    self.pitchrate = round(math.degrees(gyro[1]), 1)
                    self.yawrate = round(math.degrees(gyro[2]), 1)

                if self.yaw < 0.1:
                    self.yaw = self.yaw + 360
                if self.yaw > 360:
                    self.yaw = self.yaw - 360

                    # Dampening functions
                    self.roll_total = self.roll_total - self.roll_run[self.t_one]
                    self.roll_run[self.t_one] = self.roll
                    self.roll_total = self.roll_total + self.roll_run[self.t_one]
                    roll = round(self.roll_total / 10, 1)
                    self.heading_cos_total = self.heading_cos_total - self.heading_cos_run[self.t_three]
                    self.heading_sin_total = self.heading_sin_total - self.heading_sin_run[self.t_three]
                    self.heading_cos_run[self.t_three] = math.cos(math.radians(self.yaw))
                    self.heading_sin_run[self.t_three] = math.sin(math.radians(self.yaw))
                    self.heading_cos_total = self.heading_cos_total + self.heading_cos_run[self.t_three]
                    self.heading_sin_total = self.heading_sin_total + self.heading_sin_run[self.t_three]
                    self.yaw = round(math.degrees(math.atan2(self.heading_sin_total/30,self.heading_cos_total/30)),1)

                    if self.yaw < 0.1:
                        self.yaw = self.yaw + 360.0

                    # yaw is magnetic heading, convert to true heading
                    self.heading = self.yaw - self.magnetic_deviation

                    if self.heading < 0.1:
                        self.heading = self.heading + 360
                if self.heading > 360:
                    self.heading = self.heading - 360

                    self.t_damp = self.hack
                    self.t_one += 1
                    if self.t_one == 10:
                        self.t_one = 0
                    self.t_three += 1
                    if self.t_three == 30:
                        self.t_three = 0

                        self.t_print = self.hack

                logging.debug('IMU Reading (Heading, Roll, Pitch): ' + str(self.get_imu_reading()))

                self._data['reading'] = self.get_imu_reading()

                for callback in self.callbacks:
                    callback(self._data)

                time.sleep(self.poll_interval*1.0/1000.0)

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    imu = IMU()
    imu.initialize()
    imu.run()
