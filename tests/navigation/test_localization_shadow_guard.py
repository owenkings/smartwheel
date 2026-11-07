import math

import pytest

from wc_navigation.autonomy import shadow_localization as sl
from wc_navigation.autonomy.bringup import RunError


class FakeTransform:
    def __init__(self, x, y, yaw):
        from types import SimpleNamespace
        self.header = SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=0))
        self.transform = SimpleNamespace(translation=SimpleNamespace(x=x, y=y, z=0.0),
                                         rotation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(yaw / 2),
                                                                  w=math.cos(yaw / 2)))


class FakeBuffer:
    def __init__(self, pose=None):
        self.pose = pose

    def lookup_transform(self, target, source, time):
        if self.pose is None:
            raise LookupError("no transform")
        return FakeTransform(*self.pose)


class FakeRecorder:
    def __init__(self, pose):
        self.buffer = FakeBuffer(pose)

    def spin(self, seconds):
        pass


class FakeSupervisor:
    def __init__(self):
        self.report = {}


ALIGNMENT = {"x_m": -0.075, "y_m": 1.6, "yaw_deg": -85.0}


def test_amcl_mode_refuses_a_map_to_odom_published_before_the_start_pose():
    supervisor = FakeSupervisor()
    sl.check_single_map_to_odom(supervisor, FakeRecorder(None), {"localization_mode": "amcl"}, settle_s=0)
    assert supervisor.report["map_to_odom_publisher_check"] == "PASS"
    with pytest.raises(RunError, match="another publisher"):
        sl.check_single_map_to_odom(FakeSupervisor(), FakeRecorder((0.0, 0.0, 0.0)), {"localization_mode": "amcl"},
                                    settle_s=0)


def test_fixed_mode_requires_exactly_the_alignment():
    config = {"localization_mode": "fixed", "map_alignment": ALIGNMENT}
    sl.check_single_map_to_odom(FakeSupervisor(), FakeRecorder((-0.075, 1.6, math.radians(-85.0))), config, settle_s=0)
    for pose in (None, (0.0, 0.0, 0.0), (-0.075, 1.6, 0.0)):
        with pytest.raises(RunError):
            sl.check_single_map_to_odom(FakeSupervisor(), FakeRecorder(pose), config, settle_s=0)
