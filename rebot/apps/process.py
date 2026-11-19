"""Review continuous recordings and export annotated episodes without changing raw data."""

import argparse
import json
from pathlib import Path


def main(argv=None, prog=None):
    parser = argparse.ArgumentParser(prog=prog, description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True, help="Finalized continuous recordings")
    parser.add_argument("--output", type=Path, required=True, help="New output directory for LOOM LeRobot v3.0 camera layouts")
    parser.add_argument("--plan", type=Path, help="Editable JSON plan outside raw dataset")
    parser.add_argument("--export", action="store_true", help="Export the saved plan without opening a server")
    parser.add_argument("--port", type=int, default=8090)
    args = parser.parse_args(argv)

    from rebot.data.process import Workspace, serve

    plan = args.plan or args.output.with_name(args.output.name + ".plan.json")
    workspace = Workspace(args.dataset, args.output, plan)
    if args.export:
        print(json.dumps(workspace.export(workspace.load_plan()), indent=2))
    else:
        serve(workspace, args.port)
