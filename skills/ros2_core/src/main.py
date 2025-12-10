"""ROS2 Core Skill - Main Implementation."""

import os
import subprocess
from pathlib import Path
from typing import Dict, List, Optional


class ROS2CoreSkill:
    """Provides ROS2 environment setup and workspace management capabilities."""

    def __init__(self):
        """Initialize the ROS2 Core skill."""
        self.supported_distros = ["jazzy", "iron", "humble"]

    def setup_ros2_environment(
        self, ros_distro: str, workspace_name: str = "ros2_ws"
    ) -> Dict[str, str]:
        """
        Setup ROS2 environment with workspace.

        Args:
            ros_distro: ROS2 distribution (jazzy, iron, humble)
            workspace_name: Name of the workspace

        Returns:
            Dictionary with workspace_path and sourcing_command

        Raises:
            ValueError: If ros_distro is not supported
            RuntimeError: If ROS2 installation not found
        """
        if ros_distro not in self.supported_distros:
            raise ValueError(f"Unsupported ROS distro: {ros_distro}")

        # Check if ROS2 is installed
        ros_setup_path = Path(f"/opt/ros/{ros_distro}/setup.bash")
        if not ros_setup_path.exists():
            raise RuntimeError(f"ROS2 {ros_distro} not installed")

        # Create workspace
        workspace_path = Path.home() / workspace_name
        workspace_path.mkdir(parents=True, exist_ok=True)
        (workspace_path / "src").mkdir(exist_ok=True)

        return {
            "workspace_path": str(workspace_path),
            "sourcing_command": f"source {ros_setup_path}",
            "status": "success",
        }

    def create_ros2_package(
        self,
        workspace_path: str,
        package_name: str,
        dependencies: Optional[List[str]] = None,
        build_type: str = "ament_python",
    ) -> Dict[str, str]:
        """
        Create a new ROS2 package.

        Args:
            workspace_path: Path to ROS2 workspace
            package_name: Name of the package
            dependencies: List of package dependencies
            build_type: Build type (ament_python, ament_cmake)

        Returns:
            Dictionary with package_path and status
        """
        workspace = Path(workspace_path)
        if not workspace.exists():
            raise FileNotFoundError(f"Workspace not found: {workspace_path}")

        src_dir = workspace / "src"
        package_path = src_dir / package_name

        if package_path.exists():
            return {"package_path": str(package_path), "status": "already_exists"}

        # Build ros2 pkg create command
        cmd = ["ros2", "pkg", "create", package_name, "--build-type", build_type]
        if dependencies:
            cmd.extend(["--dependencies"] + dependencies)

        # Execute in src directory
        subprocess.run(cmd, cwd=src_dir, check=True)

        return {"package_path": str(package_path), "status": "created"}

    def build_workspace(
        self,
        workspace_path: str,
        packages: Optional[List[str]] = None,
        parallel_jobs: int = 4,
    ) -> Dict[str, bool]:
        """
        Build ROS2 workspace with colcon.

        Args:
            workspace_path: Path to ROS2 workspace
            packages: Specific packages to build (None = all)
            parallel_jobs: Number of parallel build jobs

        Returns:
            Dictionary with build_success status
        """
        workspace = Path(workspace_path)
        if not workspace.exists():
            raise FileNotFoundError(f"Workspace not found: {workspace_path}")

        cmd = ["colcon", "build", "--parallel-workers", str(parallel_jobs)]
        if packages:
            cmd.extend(["--packages-select"] + packages)

        result = subprocess.run(cmd, cwd=workspace, capture_output=True, text=True)

        return {
            "build_success": result.returncode == 0,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }


def main():
    """CLI entry point for ros2_core skill."""
    import argparse

    parser = argparse.ArgumentParser(description="ROS2 Core Skill CLI")
    subparsers = parser.add_subparsers(dest="command")

    # Setup environment command
    setup_parser = subparsers.add_parser("setup-environment")
    setup_parser.add_argument("--distro", required=True, choices=["jazzy", "iron"])
    setup_parser.add_argument("--workspace", default="ros2_ws")

    # Create package command
    create_parser = subparsers.add_parser("create-package")
    create_parser.add_argument("--name", required=True)
    create_parser.add_argument("--workspace", default=str(Path.home() / "ros2_ws"))

    args = parser.parse_args()

    skill = ROS2CoreSkill()

    if args.command == "setup-environment":
        result = skill.setup_ros2_environment(args.distro, args.workspace)
        print(f"Workspace created: {result['workspace_path']}")
    elif args.command == "create-package":
        result = skill.create_ros2_package(args.workspace, args.name)
        print(f"Package created: {result['package_path']}")


if __name__ == "__main__":
    main()
