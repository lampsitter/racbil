import ctypes
from ctypes import POINTER, c_float
import math


class Vector2(ctypes.Structure):
    _fields_ = [("x", c_float), ("y", c_float)]

    def __repr__(self):
        return f"({self.x}, {self.y})"


class TireModel(ctypes.Structure):
    _fields_ = [
        ("bx", c_float),
        ("by", c_float),
        ("cx", c_float),
        ("cy", c_float),
        ("dx", c_float),
        ("dy", c_float),
        ("ex", c_float),
        ("ey", c_float),
        ("vvx", c_float),
        ("vvy", c_float),
        ("vhx", c_float),
        ("vhy", c_float),
        ("peak_slip_x", c_float),
        ("peak_slip_y", c_float),
    ]

    def force(self, normal_force, slip_ratio, slip_angle, friction_coefficient):
        lib = ctypes.cdll.LoadLibrary("../builddir/libc_racbil.so")
        force = lib.tiremodel_force
        force.argtypes = [POINTER(TireModel), c_float, c_float, c_float, c_float]
        force.restype = Vector2
        return force(
            ctypes.byref(self),
            normal_force,
            slip_ratio,
            slip_angle,
            friction_coefficient,
        )
