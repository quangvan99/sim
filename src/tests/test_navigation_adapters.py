from __future__ import annotations

import unittest

from navigation.custom_adapter import CustomNavigationAdapter
from navigation.nav2_adapter import Nav2NavigationAdapter
from navigation.types import Point2D


STATUS = {
    "state": "IDLE",
    "message": "Ready",
    "owns_control": False,
    "start": None,
    "goal": None,
    "distance_to_goal_m": None,
    "path_length_m": None,
    "waypoint_count": 0,
    "global_path": [],
    "command": {"linear": 0.0, "angular": 0.0},
    "speed_mps": 0.5,
    "minimum_clearance_m": None,
    "error": None,
}


class FakeNavigator:
    owns_control = False

    def __init__(self):
        self.calls = []

    def plan(self, goal):
        self.calls.append(("plan", goal))
        return STATUS

    def is_goal_free(self, goal):
        self.calls.append(("validate", goal))
        return True

    def start(self):
        self.calls.append(("start",))
        return STATUS

    def cancel(self):
        self.calls.append(("cancel",))
        return STATUS

    def reset(self):
        self.calls.append(("reset",))
        return STATUS

    def set_speed_mps(self, speed):
        self.calls.append(("speed", speed))
        return STATUS

    def status(self):
        self.calls.append(("status",))
        return STATUS

    def close(self):
        self.calls.append(("close",))


class FakeGateway:
    def __init__(self):
        self.calls = []

    def get(self, path):
        self.calls.append(("GET", path, None))
        return STATUS

    def post(self, path, payload):
        self.calls.append(("POST", path, payload))
        return STATUS | {"valid": True}


class FakeTransport:
    def __init__(self, reset_result=True):
        self.reset_result = reset_result
        self.reset_calls = 0

    def reset(self):
        self.reset_calls += 1
        return self.reset_result


class NavigationAdapterTest(unittest.TestCase):
    def test_custom_adapter_delegates_without_changing_contract(self):
        navigator = FakeNavigator()
        adapter = CustomNavigationAdapter(navigator)
        adapter.plan(Point2D(1.0, 2.0))
        self.assertTrue(adapter.is_goal_free(Point2D(1.0, 2.0)))
        adapter.start()
        adapter.cancel()
        adapter.reset()
        adapter.set_speed_mps(0.4)
        adapter.status()
        adapter.close()
        self.assertEqual(navigator.calls[0], ("plan", Point2D(1.0, 2.0)))
        self.assertEqual(navigator.calls[-1], ("close",))

    def test_nav2_adapter_maps_json_calls(self):
        gateway = FakeGateway()
        transport = FakeTransport()
        adapter = Nav2NavigationAdapter(
            transport,
            "http://nav2:8010",
            gateway=gateway,
        )
        adapter.plan(Point2D(1.0, 2.0))
        self.assertTrue(adapter.is_goal_free(Point2D(1.0, 2.0)))
        adapter.start()
        adapter.cancel()
        adapter.set_speed_mps(0.4)
        adapter.status()
        adapter.reset()
        adapter.close()
        paths = [call[1] for call in gateway.calls]
        self.assertIn("/plan", paths)
        self.assertIn("/validate-goal", paths)
        self.assertIn("/reset", paths)
        self.assertEqual(transport.reset_calls, 1)

    def test_nav2_adapter_reports_gazebo_reset_failure(self):
        adapter = Nav2NavigationAdapter(
            FakeTransport(reset_result=False),
            "http://nav2:8010",
            gateway=FakeGateway(),
        )
        with self.assertRaises(RuntimeError):
            adapter.reset()


if __name__ == "__main__":
    unittest.main()
