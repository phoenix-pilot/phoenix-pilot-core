/*
 * Phoenix-Pilot
 *
 * mavlink_enums.h
 *
 * Mavlink protocol enumerators
 *
 * Copyright 2024 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-Pilot software
 *
 * %LICENSE%
 */

#ifndef PHOENIX_MAVLINK_ENUMS_H
#define PHOENIX_MAVLINK_ENUMS_H

/* Micro air vehicle / autopilot classes */
#define MAV_AUTOPILOT_GENERIC                                      0  /*	Generic autopilot, full support for everything */
#define MAV_AUTOPILOT_RESERVED                                     1  /*	Reserved for future use. */
#define MAV_AUTOPILOT_SLUGS                                        2  /*	SLUGS autopilot, http://slugsuav.soe.ucsc.edu */
#define MAV_AUTOPILOT_ARDUPILOTMEGA                                3  /*	ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org */
#define MAV_AUTOPILOT_OPENPILOT                                    4  /*	OpenPilot, http://openpilot.org */
#define MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY                       5  /*	Generic autopilot only supporting simple waypoints */
#define MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY 6  /*	Generic autopilot supporting waypoints and other simple navigation commands */
#define MAV_AUTOPILOT_GENERIC_MISSION_FULL                         7  /*	Generic autopilot supporting the full mission command set */
#define MAV_AUTOPILOT_INVALID                                      8  /*	No valid autopilot, e.g. a GCS or other MAVLink component */
#define MAV_AUTOPILOT_PPZ                                          9  /*	PPZ UAV - http://nongnu.org/paparazzi */
#define MAV_AUTOPILOT_UDB                                          10 /*	UAV Dev Board */
#define MAV_AUTOPILOT_FP                                           11 /*	FlexiPilot */
#define MAV_AUTOPILOT_PX4                                          12 /*	PX4 Autopilot - http://px4.io/ */
#define MAV_AUTOPILOT_SMACCMPILOT                                  13 /*	SMACCMPilot - http://smaccmpilot.org */
#define MAV_AUTOPILOT_AUTOQUAD                                     14 /*	AutoQuad -- http://autoquad.org */
#define MAV_AUTOPILOT_ARMAZILA                                     15 /*	Armazila -- http://armazila.com */
#define MAV_AUTOPILOT_AEROB                                        16 /*	Aerob -- http://aerob.ru */
#define MAV_AUTOPILOT_ASLUAV                                       17 /*	ASLUAV autopilot -- http://www.asl.ethz.ch */
#define MAV_AUTOPILOT_SMARTAP                                      18 /*	SmartAP Autopilot - http://sky-drones.com */
#define MAV_AUTOPILOT_AIRRAILS                                     19 /*	AirRails - http://uaventure.com */
#define MAV_AUTOPILOT_REFLEX                                       20 /*	Fusion Reflex - https://fusion.engineering */

/* MAVLINK component type reported in HEARTBEAT message. */
#define MAV_TYPE_GENERIC                   0  /* Generic micro air vehicle */
#define MAV_TYPE_FIXED_WING                1  /* Fixed wing aircraft. */
#define MAV_TYPE_QUADROTOR                 2  /* Quadrotor */
#define MAV_TYPE_COAXIAL                   3  /* Coaxial helicopter */
#define MAV_TYPE_HELICOPTER                4  /* Normal helicopter with tail rotor. */
#define MAV_TYPE_ANTENNA_TRACKER           5  /* Ground installation */
#define MAV_TYPE_GCS                       6  /* Operator control unit / ground control station */
#define MAV_TYPE_AIRSHIP                   7  /* Airship, controlled */
#define MAV_TYPE_FREE_BALLOON              8  /* Free balloon, uncontrolled */
#define MAV_TYPE_ROCKET                    9  /* Rocket */
#define MAV_TYPE_GROUND_ROVER              10 /* Ground rover */
#define MAV_TYPE_SURFACE_BOAT              11 /* Surface vessel, boat, ship */
#define MAV_TYPE_SUBMARINE                 12 /* Submarine */
#define MAV_TYPE_HEXAROTOR                 13 /* Hexarotor */
#define MAV_TYPE_OCTOROTOR                 14 /* Octorotor */
#define MAV_TYPE_TRICOPTER                 15 /* Tricopter */
#define MAV_TYPE_FLAPPING_WING             16 /* Flapping wing */
#define MAV_TYPE_KITE                      17 /* Kite */
#define MAV_TYPE_ONBOARD_CONTROLLER        18 /* Onboard companion controller */
#define MAV_TYPE_VTOL_TAILSITTER_DUOROTOR  19 /* Two-rotor Tailsitter VTOL that additionally uses control surfaces in vertical operation. Note, value previously named MAV_TYPE_VTOL_DUOROTOR. */
#define MAV_TYPE_VTOL_TAILSITTER_QUADROTOR 20 /* Quad-rotor Tailsitter VTOL using a V-shaped quad config in vertical operation. Note: value previously named MAV_TYPE_VTOL_QUADROTOR. */
#define MAV_TYPE_VTOL_TILTROTOR            21 /* Tiltrotor VTOL. Fuselage and wings stay (nominally) horizontal in all flight phases. It able to tilt (some) rotors to provide thrust in cruise flight. */
#define MAV_TYPE_VTOL_FIXEDROTOR           22 /* VTOL with separate fixed rotors for hover and cruise flight. Fuselage and wings stay (nominally) horizontal in all flight phases. */
#define MAV_TYPE_VTOL_TAILSITTER           23 /* Tailsitter VTOL. Fuselage and wings orientation changes depending on flight phase: vertical for hover, horizontal for cruise. Use more specific VTOL MAV_TYPE_VTOL_TAILSITTER_DUOROTOR or MAV_TYPE_VTOL_TAILSITTER_QUADROTOR if appropriate. */
#define MAV_TYPE_VTOL_TILTWING             24 /* Tiltwing VTOL. Fuselage stays horizontal in all flight phases. The whole wing, along with any attached engine, can tilt between vertical and horizontal mode. */
#define MAV_TYPE_VTOL_RESERVED5            25 /* VTOL reserved 5 */
#define MAV_TYPE_GIMBAL                    26 /* Gimbal */
#define MAV_TYPE_ADSB                      27 /* ADSB system */
#define MAV_TYPE_PARAFOIL                  28 /* Steerable, nonrigid airfoil */
#define MAV_TYPE_DODECAROTOR               29 /* Dodecarotor */
#define MAV_TYPE_CAMERA                    30 /* Camera */
#define MAV_TYPE_CHARGING_STATION          31 /* Charging station */
#define MAV_TYPE_FLARM                     32 /* FLARM collision avoidance system */
#define MAV_TYPE_SERVO                     33 /* Servo */
#define MAV_TYPE_ODID                      34 /* Open Drone ID. See https://mavlink.io/en/services/opendroneid.html. */
#define MAV_TYPE_DECAROTOR                 35 /* Decarotor */
#define MAV_TYPE_BATTERY                   36 /* Battery */
#define MAV_TYPE_PARACHUTE                 37 /* Parachute */
#define MAV_TYPE_LOG                       38 /* Log */
#define MAV_TYPE_OSD                       39 /* OSD */
#define MAV_TYPE_IMU                       40 /* IMU */
#define MAV_TYPE_GPS                       41 /* GPS */
#define MAV_TYPE_WINCH                     42 /* Winch */
#define MAV_TYPE_GENERIC_MULTIROTOR        43 /* Generic multirotor that does not fit into a specific type or whose type is unknown */

/* These flags encode the MAV mode. */
#define MAV_MODE_FLAG_SAFETY_ARMED         128 /* 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state. */
#define MAV_MODE_FLAG_MANUAL_INPUT_ENABLED 64  /* 0b01000000 remote control input is enabled. */
#define MAV_MODE_FLAG_HIL_ENABLED          32  /* 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational. */
#define MAV_MODE_FLAG_STABILIZE_ENABLED    16  /* 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around. */
#define MAV_MODE_FLAG_GUIDED_ENABLED       8   /* 0b00001000 guided mode enabled, system flies waypoints / mission items. */
#define MAV_MODE_FLAG_AUTO_ENABLED         4   /* 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation. */
#define MAV_MODE_FLAG_TEST_ENABLED         2   /* 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations. */
#define MAV_MODE_FLAG_CUSTOM_MODE_ENABLED  1   /* 0b00000001 Reserved for future use. */


/* These flags encode the MAV mode. These defines are predefined OR-combined mode flags */
#define MAV_MODE_PREFLIGHT          0   /* System is not ready to fly, booting, calibrating, etc. No flag is set.*/
#define MAV_MODE_STABILIZE_DISARMED 80  /* System is allowed to be active, under assisted RC control.*/
#define MAV_MODE_STABILIZE_ARMED    208 /* System is allowed to be active, under assisted RC control.*/
#define MAV_MODE_MANUAL_DISARMED    64  /* System is allowed to be active, under manual (RC) control, no stabilization*/
#define MAV_MODE_MANUAL_ARMED       192 /* System is allowed to be active, under manual (RC) control, no stabilization*/
#define MAV_MODE_GUIDED_DISARMED    88  /* System is allowed to be active, under autonomous control, manual setpoint*/
#define MAV_MODE_GUIDED_ARMED       216 /* System is allowed to be active, under autonomous control, manual setpoint*/
#define MAV_MODE_AUTO_DISARMED      92  /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)*/
#define MAV_MODE_AUTO_ARMED         220 /* System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)*/
#define MAV_MODE_TEST_DISARMED      66  /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.*/
#define MAV_MODE_TEST_ARMED         194 /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.*/


/* These values encode the bit positions of the decode position */
#define MAV_MODE_FLAG_DECODE_POSITION_SAFETY      128 /* First bit: 10000000 */
#define MAV_MODE_FLAG_DECODE_POSITION_MANUAL      64  /* Second bit: 01000000 */
#define MAV_MODE_FLAG_DECODE_POSITION_HIL         32  /* Third bit: 00100000 */
#define MAV_MODE_FLAG_DECODE_POSITION_STABILIZE   16  /* Fourth bit: 00010000 */
#define MAV_MODE_FLAG_DECODE_POSITION_GUIDED      8   /* Fifth bit: 00001000 */
#define MAV_MODE_FLAG_DECODE_POSITION_AUTO        4   /* Sixth bit: 00000100 */
#define MAV_MODE_FLAG_DECODE_POSITION_TEST        2   /* Seventh bit: 00000010 */
#define MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE 1   /* Eighth bit: 00000001 */


#define MAV_STATE_UNINIT             0 /*Uninitialized system, state is unknown. */
#define MAV_STATE_BOOT               1 /*System is booting up. */
#define MAV_STATE_CALIBRATING        2 /*System is calibrating and not flight-ready. */
#define MAV_STATE_STANDBY            3 /*System is grounded and on standby. It can be launched any time. */
#define MAV_STATE_ACTIVE             4 /*System is active and might be already airborne. Motors are engaged. */
#define MAV_STATE_CRITICAL           5 /*System is in a non-normal flight mode (failsafe). It can however still navigate. */
#define MAV_STATE_EMERGENCY          6 /*System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down. */
#define MAV_STATE_POWEROFF           7 /*System just initialized its power-down sequence, will shut down now. */
#define MAV_STATE_FLIGHT_TERMINATION 8 /*System is terminating itself (failsafe or commanded). */


#define MAV_COMP_ID_ALL                      0   /* Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message. */
#define MAV_COMP_ID_AUTOPILOT1               1   /* System flight controller component ("autopilot"). Only one autopilot is expected in a particular system. */
#define MAV_COMP_ID_USER1                    25  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER2                    26  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER3                    27  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER4                    28  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER5                    29  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER6                    30  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER7                    31  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER8                    32  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER9                    33  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER10                   34  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER11                   35  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER12                   36  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER13                   37  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER14                   38  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER15                   39  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER16                   40  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER17                   41  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER18                   42  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER19                   43  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER20                   44  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER21                   45  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER22                   46  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER23                   47  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER24                   48  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER25                   49  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER26                   50  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER27                   51  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER28                   52  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER29                   53  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER30                   54  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER31                   55  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER32                   56  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER33                   57  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER34                   58  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER35                   59  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER36                   60  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER37                   61  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER38                   62  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER39                   63  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER40                   64  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER41                   65  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER42                   66  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER43                   67  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_TELEMETRY_RADIO          68  /* Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages). */
#define MAV_COMP_ID_USER45                   69  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER46                   70  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER47                   71  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER48                   72  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER49                   73  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER50                   74  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER51                   75  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER52                   76  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER53                   77  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER54                   78  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER55                   79  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER56                   80  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER57                   81  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER58                   82  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER59                   83  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER60                   84  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER61                   85  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER62                   86  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER63                   87  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER64                   88  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER65                   89  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER66                   90  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER67                   91  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER68                   92  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER69                   93  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER70                   94  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER71                   95  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER72                   96  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER73                   97  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER74                   98  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_USER75                   99  /* Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network. */
#define MAV_COMP_ID_CAMERA                   100 /* Camera #1. */
#define MAV_COMP_ID_CAMERA2                  101 /* Camera #2. */
#define MAV_COMP_ID_CAMERA3                  102 /* Camera #3. */
#define MAV_COMP_ID_CAMERA4                  103 /* Camera #4. */
#define MAV_COMP_ID_CAMERA5                  104 /* Camera #5. */
#define MAV_COMP_ID_CAMERA6                  105 /* Camera #6. */
#define MAV_COMP_ID_SERVO1                   140 /* Servo #1. */
#define MAV_COMP_ID_SERVO2                   141 /* Servo #2. */
#define MAV_COMP_ID_SERVO3                   142 /* Servo #3. */
#define MAV_COMP_ID_SERVO4                   143 /* Servo #4. */
#define MAV_COMP_ID_SERVO5                   144 /* Servo #5. */
#define MAV_COMP_ID_SERVO6                   145 /* Servo #6. */
#define MAV_COMP_ID_SERVO7                   146 /* Servo #7. */
#define MAV_COMP_ID_SERVO8                   147 /* Servo #8. */
#define MAV_COMP_ID_SERVO9                   148 /* Servo #9. */
#define MAV_COMP_ID_SERVO10                  149 /* Servo #10. */
#define MAV_COMP_ID_SERVO11                  150 /* Servo #11. */
#define MAV_COMP_ID_SERVO12                  151 /* Servo #12. */
#define MAV_COMP_ID_SERVO13                  152 /* Servo #13. */
#define MAV_COMP_ID_SERVO14                  153 /* Servo #14. */
#define MAV_COMP_ID_GIMBAL                   154 /* Gimbal #1. */
#define MAV_COMP_ID_LOG                      155 /* Logging component. */
#define MAV_COMP_ID_ADSB                     156 /* Automatic Dependent Surveillance-Broadcast (ADS-B) component. */
#define MAV_COMP_ID_OSD                      157 /* On Screen Display (OSD) devices for video links. */
#define MAV_COMP_ID_PERIPHERAL               158 /* Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice. */
#define MAV_COMP_ID_QX1_GIMBAL               159 /* DEPRECATED: Replaced by MAV_COMP_ID_GIMBAL (2018-11). All gimbals should use MAV_COMP_ID_GIMBAL. Gimbal ID for QX1. */
#define MAV_COMP_ID_FLARM                    160 /* FLARM collision alert component. */
#define MAV_COMP_ID_PARACHUTE                161 /* Parachute component. */
#define MAV_COMP_ID_WINCH                    169 /* Winch component. */
#define MAV_COMP_ID_GIMBAL2                  171 /* Gimbal #2. */
#define MAV_COMP_ID_GIMBAL3                  172 /* Gimbal #3. */
#define MAV_COMP_ID_GIMBAL4                  173 /* Gimbal #4 */
#define MAV_COMP_ID_GIMBAL5                  174 /* Gimbal #5. */
#define MAV_COMP_ID_GIMBAL6                  175 /* Gimbal #6. */
#define MAV_COMP_ID_BATTERY                  180 /* Battery #1. */
#define MAV_COMP_ID_BATTERY2                 181 /* Battery #2. */
#define MAV_COMP_ID_MAVCAN                   189 /* CAN over MAVLink client. */
#define MAV_COMP_ID_MISSIONPLANNER           190 /* Component that can generate/supply a mission flight plan (e.g. GCS or developer API). */
#define MAV_COMP_ID_ONBOARD_COMPUTER         191 /* Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. */
#define MAV_COMP_ID_ONBOARD_COMPUTER2        192 /* Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. */
#define MAV_COMP_ID_ONBOARD_COMPUTER3        193 /* Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. */
#define MAV_COMP_ID_ONBOARD_COMPUTER4        194 /* Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on. */
#define MAV_COMP_ID_PATHPLANNER              195 /* Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.). */
#define MAV_COMP_ID_OBSTACLE_AVOIDANCE       196 /* Component that plans a collision free path between two points. */
#define MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY 197 /* Component that provides position estimates using VIO techniques. */
#define MAV_COMP_ID_PAIRING_MANAGER          198 /* Component that manages pairing of vehicle and GCS. */
#define MAV_COMP_ID_IMU                      200 /* Inertial Measurement Unit (IMU) #1. */
#define MAV_COMP_ID_IMU_2                    201 /* Inertial Measurement Unit (IMU) #2. */
#define MAV_COMP_ID_IMU_3                    202 /* Inertial Measurement Unit (IMU) #3. */
#define MAV_COMP_ID_GPS                      220 /* GPS #1. */
#define MAV_COMP_ID_GPS2                     221 /* GPS #2. */
#define MAV_COMP_ID_ODID_TXRX_1              236 /* Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet). */
#define MAV_COMP_ID_ODID_TXRX_2              237 /* Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet). */
#define MAV_COMP_ID_ODID_TXRX_3              238 /* Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet). */
#define MAV_COMP_ID_UDP_BRIDGE               240 /* Component to bridge MAVLink to UDP (i.e. from a UART). */
#define MAV_COMP_ID_UART_BRIDGE              241 /* Component to bridge to UART (i.e. from UDP). */
#define MAV_COMP_ID_TUNNEL_NODE              242 /* Component handling TUNNEL messages (e.g. vendor specific GUI of a component). */
#define MAV_COMP_ID_SYSTEM_CONTROL           250 /* DEPRECATED: Replaced by MAV_COMP_ID_ALL (2018-11). System control does not require a separate component ID. Instead, system commands should be sent with target_component=MAV_COMP_ID_ALL allowing the target component to use any appropriate component id. */

#endif
