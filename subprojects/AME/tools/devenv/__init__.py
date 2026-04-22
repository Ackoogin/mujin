r"""
Quick rclpy service call test - bypasses the ros2 CLI.
Run from a shell that has ROS2 + ame_ros2 sourced:

  call D:\Dev\ros2-windows\setup.bat
  call D:\Dev\repo\unmanned\install\setup.bat
  set PYTHONPATH=%PYTHONPATH%;D:\Dev\ros2-windows\.pixi\envs\default\Lib\site-packages
  set CYCLONEDDS_URI=file://D:/Dev/repo/unmanned/cyclonedds_localhost.xml
  D:\Dev\repo\unmanned\subprojects\AME\tools\devenv\.venv\Scripts\python.exe subprojects\AME\tools\devenv\test_rclpy_svc.py
"""

import sys
import time

try:
    import rclpy
    from rclpy.node import Node
except ImportError as e:
    print(f"[ERROR] rclpy not importable: {e}")
    print("  Make sure PYTHONPATH includes the pixi site-packages.")
    sys.exit(1)

try:
    from ame_ros2.srv import QueryState
except ImportError as e:
    print(f"[ERROR] ame_ros2 messages not importable: {e}")
    print("  Make sure install\\setup.bat has been called.")
    sys.exit(1)


def main():
    rclpy.init()
    node = Node("rclpy_svc_test")
    print(f"[INFO] Node created: {node.get_name()}")

    client = node.create_client(QueryState, "/world_model_node/query_state")
    print("[INFO] Waiting for /world_model_node/query_state service (10 s)...")

    if not client.wait_for_service(timeout_sec=10.0):
        print("[ERROR] Service not available after 10 s — is ame_combined running?")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print("[INFO] Service found — sending request...")
    req = QueryState.Request()
    req.keys = []  # empty = return all true facts

    future = client.call_async(req)
    t0 = time.time()

    while not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - t0 > 10.0:
            print("[ERROR] Timed out waiting for response (10 s)")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    result = future.result()
    print(f"[OK] Response received in {time.time()-t0:.2f}s")
    print(f"     facts ({len(result.facts)} total):")
    for f in result.facts:
        print(f"       {f}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
