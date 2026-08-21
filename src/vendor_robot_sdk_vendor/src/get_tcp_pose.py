#!/usr/bin/env python3
"""Read the actual TCP pose through the vendor SDK or MoveIt TF.

The reported pose is ``[x, y, z, rx, ry, rz]`` in mm and degrees, matching
``cr_get_tcpActualPose`` in ``robotapi.h``.  This script is read-only: it never
sends a robot motion command.
"""

import argparse
import ctypes
import sys
import time
from pathlib import Path


SUCCESS = 0
AXIS_COUNT = 6


def sdk_error_message(result: int) -> str:
    """Return the SDK meanings used by this small read-only client."""
    messages = {
        1: "general error",
        2: "connection already exists",
        3: "operation timed out",
        4: "invalid RPC result",
        7: "invalid parameter",
        10: "robot handle was not created",
        17: "operation rejected in the current robot mode",
    }
    return messages.get(result, "SDK error")


def load_sdk(library_dir: Path) -> ctypes.CDLL:
    """Load SDK dependencies from the package's lib directory.

    The supplied SDK has no RUNPATH, so dependencies are preloaded with global
    visibility instead of relying on the caller having set LD_LIBRARY_PATH.
    """
    dependency_names = (
        "libz.so.1",
        "libprotobuf-c.so.1",
        "libprotobuf-rpc.so.1",
        "libCGXIZip.so.1",
        "libLogStorageDll.so.1",
        "libRobotConfigDll.so.1",
        "libtpAlgApp.so.1",
    )
    try:
        for name in dependency_names:
            ctypes.CDLL(str(library_dir / name), mode=ctypes.RTLD_GLOBAL)
        return ctypes.CDLL(str(library_dir / "libcr_sdk.so"), mode=ctypes.RTLD_GLOBAL)
    except OSError as error:
        raise RuntimeError(
            f"cannot load SDK from {library_dir}: {error}; "
            "check that the x86_64 SDK libraries are present"
        ) from error


def configure_functions(sdk: ctypes.CDLL) -> None:
    sdk.cr_create_robot.argtypes = (
        ctypes.POINTER(ctypes.c_int), ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p)
    sdk.cr_create_robot.restype = ctypes.c_int
    sdk.cr_get_tcpActualPose.argtypes = (ctypes.c_int, ctypes.POINTER(ctypes.c_double))
    sdk.cr_get_tcpActualPose.restype = ctypes.c_int
    sdk.cr_destroy_robot.argtypes = (ctypes.c_int,)
    sdk.cr_destroy_robot.restype = ctypes.c_int


def read_sdk_pose(args: argparse.Namespace) -> int:
    """Read the vendor controller's actual TCP pose in mm/degrees."""
    try:
        sdk = load_sdk(args.library_dir)
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    configure_functions(sdk)

    handle = ctypes.c_int(-1)
    result = sdk.cr_create_robot(
        ctypes.byref(handle), args.robot_ip.encode(), args.sdk_port, args.password.encode())
    if result != SUCCESS:
        print(f"ERROR: SDK connection to {args.robot_ip}:{args.sdk_port} failed "
              f"({result}: {sdk_error_message(result)})", file=sys.stderr)
        return 1

    try:
        pose = (ctypes.c_double * AXIS_COUNT)()
        result = sdk.cr_get_tcpActualPose(handle, pose)
        if result != SUCCESS:
            print(f"ERROR: read TCP pose failed ({result}: {sdk_error_message(result)})", file=sys.stderr)
            return 1
        x, y, z, rx, ry, rz = pose
        print(f"SDK TCP pose [mm, deg]: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
              f"rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
        return 0
    finally:
        if handle.value >= 0:
            destroy_result = sdk.cr_destroy_robot(handle)
            if destroy_result != SUCCESS:
                print(f"WARNING: SDK disconnect failed ({destroy_result}: "
                      f"{sdk_error_message(destroy_result)})", file=sys.stderr)


def read_moveit_pose(args: argparse.Namespace) -> int:
    """Read the FK pose published by MoveIt/robot_state_publisher via TF2.

    This is model-based FK from the current ROS joint state, not a controller
    measurement. It therefore uses metres and a quaternion, as required by ROS.
    """
    try:
        import rclpy
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as error:
        print(f"ERROR: MoveIt/ROS 2 Python dependencies are unavailable: {error}", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("get_tcp_pose", namespace=args.namespace.strip("/"))
    buffer = Buffer()
    # Keep spinning in this function rather than using TransformListener's
    # background executor.  That avoids ExternalShutdownException being printed
    # by the listener thread when this one-shot CLI calls rclpy.shutdown().
    listener = TransformListener(buffer, node, spin_thread=False)
    try:
        deadline = time.monotonic() + args.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                transform = buffer.lookup_transform(
                    args.base_frame, args.end_effector_frame, Time(),
                    timeout=Duration(seconds=0.2))
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                print("MoveIt TF pose [m, quaternion]: "
                      f"frame={args.base_frame}, link={args.end_effector_frame}, "
                      f"x={translation.x:.6f}, y={translation.y:.6f}, z={translation.z:.6f}, "
                      f"qx={rotation.x:.6f}, qy={rotation.y:.6f}, "
                      f"qz={rotation.z:.6f}, qw={rotation.w:.6f}")
                return 0
            except TransformException:
                rclpy.spin_once(node, timeout_sec=0.1)
        print(f"ERROR: no TF transform from '{args.base_frame}' to "
              f"'{args.end_effector_frame}' within {args.timeout:.1f} s. "
              "Start robot_state_publisher/MoveIt and verify /joint_states.", file=sys.stderr)
        return 1
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    package_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Read the robot TCP pose via the vendor SDK or MoveIt TF")
    parser.add_argument("--source", choices=("sdk", "moveit"), default="sdk",
                        help="sdk: actual controller TCP; moveit: TF forward-kinematics pose")
    parser.add_argument("--robot-ip", default="192.168.6.6", help="robot controller IP address")
    parser.add_argument("--sdk-port", type=int, default=2323, help="SDK port (2323 real, 2325 virtual)")
    parser.add_argument("--password", default="", help="SDK login password")
    parser.add_argument("--library-dir", type=Path, default=package_root / "lib",
                        help="directory containing libcr_sdk.so and its dependencies")
    parser.add_argument("--base-frame", default="base_link",
                        help="MoveIt TF source frame (default: base_link)")
    parser.add_argument("--end-effector-frame", default="Link6",
                        help="MoveIt TF end-effector link (default: Link6)")
    parser.add_argument("--namespace", default="", help="ROS namespace containing the MoveIt TF graph")
    parser.add_argument("--timeout", type=float, default=3.0,
                        help="MoveIt TF lookup timeout in seconds")
    args = parser.parse_args()

    if not 1 <= args.sdk_port <= 65535:
        parser.error("--sdk-port must be in [1, 65535]")
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    return read_sdk_pose(args) if args.source == "sdk" else read_moveit_pose(args)


if __name__ == "__main__":
    raise SystemExit(main())
