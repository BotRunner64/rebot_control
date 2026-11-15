"""Preview Linux webcams in a browser using GStreamer and automatic capture modes."""

from rebot.media import capture


def main(argv=None, prog=None):
    # The capture module parses the options, then re-executes itself under the
    # system interpreter when this environment has no GStreamer bindings.
    return capture.main(argv, prog=prog)
