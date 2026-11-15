"""The `rebot` dispatcher: exit codes, one-line errors, and unknown commands."""

from contextlib import redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import patch

import can

from rebot import cli
from rebot.apps import guide


class DispatchTests(unittest.TestCase):
    def run_cli(self, *argv):
        output, errors = io.StringIO(), io.StringIO()
        with redirect_stdout(output), redirect_stderr(errors):
            status = cli.main(list(argv))
        return status, output.getvalue(), errors.getvalue()

    def test_exit_codes_and_one_line_errors(self):
        for result, status in ((None, 0), (3, 3), (RuntimeError("Guide failed."), 2),
                               (ValueError("Bad side."), 2), (TimeoutError("No reply."), 2),
                               (OSError("No adapter."), 2), (can.CanError("Bus off."), 2),
                               (KeyboardInterrupt(), 130)):
            failure = isinstance(result, BaseException)
            with self.subTest(result=result), \
                    patch.object(guide, "main", **({"side_effect": result} if failure
                                                   else {"return_value": result})) as main:
                code, _, errors = self.run_cli("guide", "--side", "left")
            self.assertEqual(code, status)
            main.assert_called_once_with(["--side", "left"], prog="rebot guide")
            self.assertEqual(errors, f"rebot guide: {result}\n" if status == 2 else "")

    def test_unexpected_errors_are_not_swallowed(self):
        with patch.object(guide, "main", side_effect=AssertionError("bug")), \
                self.assertRaises(AssertionError):
            self.run_cli("guide")

    def test_unknown_command_and_usage(self):
        status, _, errors = self.run_cli("fly")
        self.assertEqual(status, 2)
        self.assertTrue(errors.startswith("rebot: unknown command 'fly'\n"))
        for name in cli.COMMANDS:
            self.assertIn(name, errors)
        self.assertEqual(self.run_cli("--help")[0], 0)
        self.assertEqual(self.run_cli()[0], 2)


if __name__ == "__main__":
    unittest.main()
