from collections import defaultdict
from enum import IntFlag

from cereal import car
from selfdrive.car import dbc_dict
from selfdrive.config import Conversions as CV

Ecu = car.CarParams.Ecu
MIN_ACC_SPEED = 40. * CV.KPH_TO_MS
PEDAL_TRANSITION = 10. * CV.MPH_TO_MS


class CarControllerParams:
  ACCEL_MAX = 1.5  # m/s2, lower than allowed 2.0 m/s2 for tuning reasons
  ACCEL_MIN = -3.5  # m/s2

  STEER_MAX = 1500
  STEER_DELTA_UP = 10       # 1.5s time to peak torque
  STEER_DELTA_DOWN = 25     # always lower than 45 otherwise the Rav4 faults (Prius seems ok with 50)
  STEER_ERROR_MAX = 350     # max delta between torque cmd and torque motor


class ToyotaFlags(IntFlag):
  HYBRID = 1


class CAR:
  OUTLANDER_GT = "OUTLANDER_GT_2016"

FINGERPRINTS = {
  CAR.OUTLANDER_GT: [{
    2: 8, 64: 8, 65: 8, 72: 8, 73: 8, 280: 8, 281: 8, 290: 8, 312: 8, 313: 8, 314: 8, 315: 8, 316: 8, 326: 8, 372: 8, 544: 8, 545: 8, 546: 8, 552: 8, 554: 8, 557: 8, 576: 8, 577: 8, 722: 8, 801: 8, 802: 8, 805: 8, 808: 8, 811: 8, 816: 8, 826: 8, 827: 8, 837: 8, 838: 8, 839: 8, 842: 8, 912: 8, 915: 8, 940: 8, 1614: 8, 1617: 8, 1632: 8, 1650: 8, 1657: 8, 1658: 8, 1677: 8, 1697: 8, 1722: 8, 1743: 8, 1759: 8, 1786: 5, 1787: 5, 1788: 8, 1809: 8, 1813: 8, 1817: 8, 1821: 8, 1840: 8, 1848: 8, 1924: 8, 1932: 8, 1952: 8, 1960: 8
  }]
}


# (addr, cars, bus, 1/freq*100, vl)
STATIC_DSU_MSGS = [
  (0x128, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.AVALON), 1,   3, b'\xf4\x01\x90\x83\x00\x37'),
  (0x128, (CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH), 1,   3, b'\x03\x00\x20\x00\x00\x52'),
  (0x141, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 1,   2, b'\x00\x00\x00\x46'),
  (0x160, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 1,   7, b'\x00\x00\x08\x12\x01\x31\x9c\x51'),
  (0x161, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.AVALON, CAR.LEXUS_RX, CAR.PRIUS_V), 1,   7, b'\x00\x1e\x00\x00\x00\x80\x07'),
  (0X161, (CAR.HIGHLANDERH, CAR.HIGHLANDER, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH), 1,  7, b'\x00\x1e\x00\xd4\x00\x00\x5b'),
  (0x283, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 0,   3, b'\x00\x00\x00\x00\x00\x00\x8c'),
  (0x2E6, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,   3, b'\xff\xf8\x00\x08\x7f\xe0\x00\x4e'),
  (0x2E7, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,   3, b'\xa8\x9c\x31\x9c\x00\x00\x00\x02'),
  (0x33E, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH), 0,  20, b'\x0f\xff\x26\x40\x00\x1f\x00'),
  (0x344, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 0,   5, b'\x00\x00\x01\x00\x00\x00\x00\x50'),
  (0x365, (CAR.PRIUS, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.HIGHLANDERH), 0,  20, b'\x00\x00\x00\x80\x03\x00\x08'),
  (0x365, (CAR.RAV4, CAR.RAV4H, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 0,  20, b'\x00\x00\x00\x80\xfc\x00\x08'),
  (0x366, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.HIGHLANDERH), 0,  20, b'\x00\x00\x4d\x82\x40\x02\x00'),
  (0x366, (CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDER, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 0,  20, b'\x00\x72\x07\xff\x09\xfe\x00'),
  (0x470, (CAR.PRIUS, CAR.LEXUS_RXH), 1, 100, b'\x00\x00\x02\x7a'),
  (0x470, (CAR.HIGHLANDER, CAR.HIGHLANDERH, CAR.RAV4H, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.PRIUS_V), 1,  100, b'\x00\x00\x01\x79'),
  (0x4CB, (CAR.PRIUS, CAR.RAV4H, CAR.LEXUS_RXH, CAR.LEXUS_NXH, CAR.LEXUS_NX, CAR.RAV4, CAR.COROLLA, CAR.HIGHLANDERH, CAR.HIGHLANDER, CAR.AVALON, CAR.SIENNA, CAR.LEXUS_CTH, CAR.LEXUS_ESH, CAR.LEXUS_RX, CAR.PRIUS_V), 0, 100, b'\x0c\x00\x00\x00\x00\x00\x00\x00'),
]

FW_VERSIONS = {
  CAR.OUTLANDER_GT: {
    (Ecu.eps, 0x7a1, None): [

    ],
    (Ecu.esp, 0x7b0, None): [

    ],
    (Ecu.engine, 0x700, None): [

    ],
    (Ecu.fwdRadar, 0x750, 0xf): [

    ],
    (Ecu.fwdCamera, 0x750, 0x6d): [

    ],
  }
}

STEER_THRESHOLD = 100

DBC = {
  CAR.OUTLANDER_GT: dbc_dict('outlander_gt_generated', none)
}

PREGLOBAL_CARS = [CAR.OUTLANDER_PREGLOBAL]
