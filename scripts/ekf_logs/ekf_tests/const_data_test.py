from common.models import logs_data
from common.models.utils import Vector3, GlobalPosition, NEDCoordinates

from ekf_tests.test_scenario_creator import LogsGenerator, TestScenarioCreator

import itertools


def celsius_to_milikelwins(temp):
    return int((temp + 273.15) * 1000)


def generate_const_data_scenario():
    sensors_temp = celsius_to_milikelwins(24)
    dop = 4  # good, but not perfect dilution of position

    imu = logs_data.Imu(
        accelDeviceID=1,
        accel=Vector3(0, 0, -9806),
        accelTemp=sensors_temp,

        gyroDeviceID=2,
        gyro=Vector3.Zero(),
        gyroDAngle=Vector3.Zero(),
        gyroTemp=sensors_temp,

        magDeviceID=3,
        mag=Vector3(275, -19, -161)
    )

    gps = logs_data.Gps(
        deviceID=4,

        position=GlobalPosition(
            altitude=0,
            latitude=0,
            longitude=0
        ),

        utc=0,

        horizontalPrecisionDilution=dop,
        verticalPrecisionDilution=dop,

        ellipsoidAlt=0,
        groundSpeed=0,
        velocity=NEDCoordinates(0, 0, 0),

        horizontalAccuracy=1_000,  # one meter
        verticalAccuracy=1_000,
        velocityAccuracy=1_000,

        heading=0,
        headingOffset=0,
        headingAccuracy=1_000,  # one degree

        satelliteNumber=6,
        fix=1
    )

    baro = logs_data.Baro(
        deviceID=5,
        pressure=101_325,  # normal sea level pressure
        temperature=sensors_temp
    )

    logs_generator = LogsGenerator(
        itertools.repeat(imu),
        itertools.repeat(gps),
        itertools.repeat(baro)
    )

    scenario_creator = TestScenarioCreator()
    scenario_creator.create(logs_generator, "const_data_ekf_scenario.bin", 15_000_000)
