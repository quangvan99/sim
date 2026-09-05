"""HTTP contract tests for the AMR control server."""

from __future__ import annotations

import unittest

from pydantic import ValidationError

from amr.server import VelocityCommand


class AMRServerTest(unittest.TestCase):
    def test_command_uses_explicit_strict_json_schema(self):
        command = VelocityCommand.model_validate(
            {"linear": 0.1, "angular": 0}
        )
        self.assertEqual(command.linear, 0.1)
        self.assertEqual(command.angular, 0.0)

        for payload in (
            {"linear": "0.1", "angular": 0},
            {"linear": True, "angular": 0},
            {"linear": 0.1},
            {"linear": 0.1, "angular": 0, "unexpected": 1},
        ):
            with self.subTest(payload=payload), self.assertRaises(ValidationError):
                VelocityCommand.model_validate(payload)


if __name__ == "__main__":
    unittest.main()
