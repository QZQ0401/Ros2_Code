#!/usr/bin/env python3
import argparse
import hashlib
import sys
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("calibration")
    parser.add_argument("--robot-serial", required=True)
    parser.add_argument("--kinematics-file")
    args = parser.parse_args()
    with open(args.calibration, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    errors = []
    if data.get("schema_version") != 1:
        errors.append("unsupported schema_version")
    if data.get("robot_serial") in (None, "", "UNSET"):
        errors.append("robot_serial is not bound")
    elif data["robot_serial"] != args.robot_serial:
        errors.append("robot_serial mismatch")
    parameters = data.get("kinematic_parameters", {})
    if not parameters.get("verified", False):
        errors.append("kinematics are not verified")
    if args.kinematics_file:
        with open(args.kinematics_file, "rb") as stream:
            digest = hashlib.sha256(stream.read()).hexdigest()
        if digest != parameters.get("sha256"):
            errors.append("kinematics sha256 mismatch")
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print("calibration identity and checksum are valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
