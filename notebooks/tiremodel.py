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


def default_tiremodel():
    return TireModel(
        bx=1.9,
        by=9.0,
        cx=1.65,
        cy=1.36,
        dx=1.1,
        dy=1.0,
        ex=-1.0,
        ey=0.96,
        vvx=0.0,
        vvy=0.0,
        vhx=0.0,
        vhy=0.0,
        peak_slip_x=0.4,
        peak_slip_y=math.radians(20.0),
    )
