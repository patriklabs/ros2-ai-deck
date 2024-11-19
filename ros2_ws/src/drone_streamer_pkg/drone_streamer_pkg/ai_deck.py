import struct
from enum import Enum

import cv2
import numpy as np
from cflib.cpx import CPX, CPXFunction, CPXPacket, CPXTarget
from cflib.cpx.transports import SocketTransport


class MyCPXFunction(Enum):
    CPX_F_CONTROL = 7


class IMU:

    magic = 0xBE

    def decode(self, cpx_data):

        [magic, gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, timestamp] = struct.unpack(
            "<BhhhhhhQ", cpx_data.data
        )

        if self.magic != magic:
            raise Exception("magic did not match for imu data")

        self.gyro_x = gyro_x / 500
        self.gyro_y = gyro_y / 500
        self.gyro_z = gyro_z / 500

        self.acc_x = acc_x / 1000
        self.acc_y = acc_y / 1000
        self.acc_z = acc_z / 1000
        self.timestamp = timestamp

    def print(self):
        print(
            f"{self.magic}, {self.gyro_x:.3f}, {self.gyro_y:.3f}, {self.gyro_z:.3f}, {self.acc_x:.3f}, {self.acc_y:.3f}, {self.acc_z:.3f}, {self.timestamp}"
        )

    def write(self, path):
        with open(path, "a") as f:
            f.write(
                f"{self.magic}, {self.gyro_x:.3f}, {self.gyro_y:.3f}, {self.gyro_z:.3f}, {self.acc_x:.3f}, {self.acc_y:.3f}, {self.acc_z:.3f}, {self.timestamp}\n"
            )


class ImageHeader:

    magic = 0xBC

    def decode(self, cpx_data):

        if len(cpx_data.data) != 19:
            raise Exception("image header data too long!")

        [magic, width, height, depth, format, size, timestamp] = struct.unpack(
            "<BHHBBIQ", cpx_data.data
        )

        if self.magic != magic:
            raise Exception("magic did not match for image header data")

        self.width = width
        self.height = height
        self.depth = depth
        self.format = format
        self.size = size
        self.timestamp = timestamp

    def print(self):
        print(
            f"{self.magic}, {self.width}, {self.height}, {self.depth}, {self.format}, {self.size}, {self.timestamp}"
        )


class VelocitySetpoint:

    def __init__(self, vel_x=0, vel_y=0, vel_z=0, yaw_rate=0) -> None:

        self.magic = 0xAA
        self.vel_x = int(vel_x * 1000)  # short
        self.vel_y = int(vel_y * 1000)  # short
        self.vel_z = int(vel_z * 1000)  # short
        self.yaw_rate = int(yaw_rate * 500)  # short

    def encode(self):
        return struct.pack(
            "<Bhhhh", self.magic, self.vel_x, self.vel_y, self.vel_z, self.yaw_rate
        )

    def decode(self, data):

        [self.magic, self.vel_x, self.vel_y, self.vel_z, self.yaw_rate] = struct.unpack(
            "<Bhhhh", data
        )

    def print(self):

        print(
            f"{self.magic}, {self.vel_x}, {self.vel_y}, {self.vel_z}, {self.yaw_rate}"
        )


class ImageData:

    magic = 0xAC

    def __init__(self, image_header: ImageHeader) -> None:
        self.image_header = image_header

        self.imgStream = bytearray()

    def decode(self, cpx_data):

        if cpx_data.data[0] != self.magic:
            raise Exception("magic did not match for image data")

        if len(cpx_data.data[1:]) + len(self.imgStream) <= self.image_header.size:
            self.imgStream.extend(cpx_data.data[1:])
        else:
            raise Exception("image data too long!")

    def complete(self):
        return len(self.imgStream) == self.image_header.size

    def get_image(self):
        if self.image_header.format == 0:
            bayer_img = np.frombuffer(self.imgStream, dtype=np.uint8)
            bayer_img.shape = (self.image_header.height, self.image_header.width)
            color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
            return color_img
        else:
            nparr = np.frombuffer(self.imgStream, np.uint8)
            decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)

            return decoded


from abc import ABC, abstractmethod


class AiDeckCallbackInterface(ABC):

    @abstractmethod
    def imu_callback(self, imu: IMU):
        pass

    @abstractmethod
    def image_callback(self, image: ImageData):
        pass


class AiDeck:

    def __init__(self, ip="192.168.4.1", port=5000, show_image=False):
        self.cpx = CPX(SocketTransport(ip, port))

        self.imu = IMU()
        self.image_header = ImageHeader()
        self.image_data = None
        self.show_image = show_image

    def spin_once(self, callback: AiDeckCallbackInterface = None):

        # Read the IMU data
        try:
            cpx_data = self.cpx.receivePacket(CPXFunction.APP, 0.05)
        except:
            return None

        # Decode the data

        magic = cpx_data.data[0]

        if magic == IMU.magic:

            self.imu.decode(cpx_data)

            if callback:
                callback.imu_callback(self.imu)

        elif magic == ImageHeader.magic:

            self.image_header.decode(cpx_data)
            # self.image_header.print()

            self.image_data = ImageData(self.image_header)

        elif magic == ImageData.magic:

            if self.image_data is None:
                raise Exception("image data received before image header")

            self.image_data.decode(cpx_data)

            if self.image_data.complete():

                if callback:
                    callback.image_callback(self.image_data)

                if self.show_image:
                    cv2.imshow("Image", self.image_data.get_image())
                    cv2.waitKey(1)

                self.image_data = None

    def set_velocity_setpoint(self, setpoint: VelocitySetpoint):
        cpxPacket = CPXPacket(
            MyCPXFunction.CPX_F_CONTROL,
            CPXTarget.GAP8,
            CPXTarget.HOST,
            setpoint.encode(),
        )
        self.cpx.sendPacket(cpxPacket)
