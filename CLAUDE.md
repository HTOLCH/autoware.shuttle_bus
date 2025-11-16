# CLAUDE.md - AI Assistant Guide for Autoware Shuttle Bus

## Project Overview

**Autoware Shuttle Bus** is a meta-repository for the world's leading open-source autonomous driving software stack, built on ROS 2 (Robot Operating System 2). This repository serves as a configuration and orchestration layer that brings together multiple Autoware component repositories.

- **Primary ROS Distribution**: ROS 2 Humble (default), Jazzy (experimental)
- **Platforms**: x86_64 (amd64), ARM64 (aarch64)
- **Build System**: colcon (ROS 2 standard)
- **Container Runtime**: Docker with multi-stage builds
- **Infrastructure**: Ansible-based environment setup

## Repository Architecture

### Meta-Repository Pattern

This is NOT a traditional monorepo. Source code lives in external repositories imported via `.repos` files:

```
autoware.shuttle_bus/
├── src/                    # Populated via 'vcs import' (gitignored)
│   ├── core/              # Autoware Core packages
│   ├── universe/          # Autoware Universe packages
│   ├── launcher/          # Launch configurations
│   └── sensor_component/  # Sensor drivers
├── build/                 # colcon build artifacts (gitignored)
├── install/               # colcon install artifacts (gitignored)
└── log/                   # Build logs (gitignored)
```

**Never manually edit files in `src/`** - they come from external repositories defined in:
- `autoware.repos` - Main Autoware dependencies (Core, Universe, Launcher)
- `simulator.repos` - Scenario Simulator v2
- `tools.repos` - Development tools

### Component Architecture

Autoware follows a layered architecture:

1. **Core Layer** (`autoware_core`) - Stable, high-quality fundamental packages
2. **Universe Layer** (`autoware_universe`) - Experimental, cutting-edge features
   - Sensing → Perception → Localization/Mapping → Planning → Control → System
3. **Launcher Layer** (`autoware_launch`) - Node configurations and parameters
4. **Vehicle Integration** - Vehicle-specific interfaces and sensor kits

**Custom Fork**: This repository uses `HTOLCH/nuway_autoware_launch` instead of the standard launcher.

## Development Environment Setup

### Prerequisites

- Ubuntu 22.04 (recommended) or 24.04
- Git installed
- Sudo access
- 50GB+ free disk space
- (Optional) NVIDIA GPU with drivers for CUDA support

### Initial Setup

```bash
# 1. Clone repository
git clone <repo-url>
cd autoware.shuttle_bus

# 2. Create workspace directories
mkdir -p src

# 3. Import dependencies using vcstool
vcs import src < autoware.repos
vcs import src < simulator.repos
vcs import src < tools.repos

# 4. Run automated environment setup (uses Ansible)
./setup-dev-env.sh universe  # Options: core, universe

# 5. Source ROS environment
source ~/.bashrc

# 6. Install ROS dependencies
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# 7. Build workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 8. Source workspace
source install/setup.bash
```

### Setup Script Options

The `setup-dev-env.sh` script supports:
- `-y` - Non-interactive mode (for CI)
- `--no-nvidia` - Skip CUDA/TensorRT installation
- `--runtime` - Install runtime packages only (no dev tools)
- `--data-dir <path>` - Set artifact download location (default: ~/autoware_data)
- `--ros-distro <humble|jazzy>` - Choose ROS distribution
- `--module <name>` - Install specific modules only

### Development Container (Recommended)

For VS Code users, devcontainer configurations are available:

- `.devcontainer/universe-devel-cuda/` - Full development with CUDA
- `.devcontainer/universe-devel/` - Full development without CUDA
- `.devcontainer/core-devel/` - Core packages only

**Container features**:
- Pre-configured with all dependencies
- ccache for fast rebuilds
- ROS 2 environment auto-sourced
- Debugging tools (gdb, lldb)

## Code Standards and Quality

### Pre-commit Hooks (MANDATORY)

All code MUST pass pre-commit checks before merging:

```bash
# Install pre-commit
pip install pre-commit

# Install hooks to .git/hooks/
pre-commit install

# Run manually before committing
pre-commit run --all-files
```

**Key hooks enforced**:
- `clang-format` - C++ formatting (Google style, 100 char limit)
- `cpplint` - C++ linting
- `black` - Python formatting (100 char limit)
- `isort` - Python import sorting
- `flake8-ros` - Python linting with ROS conventions
- `shellcheck` / `shfmt` - Shell script validation
- `markdownlint-cli` - Markdown formatting
- `prettier` - YAML/JSON/XML formatting
- `hadolint` - Dockerfile linting

### C++ Coding Standards

**Style Guide**: Google C++ Style with Autoware modifications

**Key rules**:
- 100 character line limit
- Snake case for variables/functions: `my_variable`, `calculate_distance()`
- Pascal case for classes: `ObjectDetector`
- Pointer alignment: middle (`int* ptr;` not `int *ptr;`)
- Include order: C++ stdlib → C headers → Boost → ROS messages → Local headers
- Brace style: Namespaces/classes on new line, functions K&R style

**Configuration files**:
- `.clang-format` - Formatting rules (applied via pre-commit)
- `.clang-tidy` - Static analysis (400+ checks)
- `CPPLINT.cfg` - cpplint configuration

**Example**:
```cpp
#include <memory>
#include <vector>

#include <boost/geometry.hpp>

#include "autoware_msgs/msg/detected_objects.hpp"

#include "my_package/object_detector.hpp"

namespace my_package
{

class ObjectDetector
{
public:
  explicit ObjectDetector(const rclcpp::NodeOptions & options);

  void detect_objects(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input);

private:
  double detection_threshold_;
  std::shared_ptr<ModelInterface> model_;
};

}  // namespace my_package
```

### Python Coding Standards

**Style Guide**: PEP 8 with Black formatting

**Key rules**:
- 100 character line limit
- Black formatter (no exceptions)
- isort for import organization (PEP 8 profile)
- Type hints strongly encouraged
- Snake case: `my_function()`, `my_variable`

**Configuration**: `setup.cfg` for flake8/isort

**Example**:
```python
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node

from autoware_msgs.msg import DetectedObjects


class ObjectProcessor(Node):
    def __init__(self) -> None:
        super().__init__('object_processor')
        self.declare_parameter('threshold', 0.5)
        self.threshold: float = self.get_parameter('threshold').value

    def process_objects(
        self, objects: DetectedObjects
    ) -> Optional[List[DetectedObject]]:
        """Process detected objects and filter by threshold."""
        return [obj for obj in objects.objects if obj.score > self.threshold]
```

### YAML/XML Standards

**YAML** (`.yamllint.yaml`):
- Consistent indentation (2 spaces)
- No document markers (`---`) required
- Truthy values allowed (for GitHub Actions)
- Special exception: `*.param.yaml` files ignored (ROS parameters)

**XML** (ROS launch/xacro/package files):
- Prettier formatting with XML plugin
- Print width: 200 chars for launch/xacro, 1000 for package.xml
- Self-closing tags: `<node .../>`
- Alphabetically sorted attributes

### Shell Script Standards

- Bash shebang: `#!/usr/bin/env bash`
- shellcheck validation (no errors allowed)
- shfmt formatting (4-space indent)
- Set error handling: `set -e` for critical scripts

## Git Workflow

### Branch Strategy

- `main` - Stable branch, synced with upstream autowarefoundation/autoware
- Feature branches: `feature/description` or `claude/session-id`
- Never commit directly to main without PR

### Commit Message Format (MANDATORY)

Commits MUST follow **Conventional Commits** specification:

```
<type>(<scope>): <subject>

<body>

<footer>
```

**Types**:
- `feat:` - New feature
- `fix:` - Bug fix
- `docs:` - Documentation changes
- `style:` - Code style changes (formatting, no logic change)
- `refactor:` - Code refactoring (no feature/fix)
- `perf:` - Performance improvements
- `test:` - Adding/updating tests
- `chore:` - Build process, tooling changes
- `ci:` - CI/CD configuration changes

**Examples**:
```
feat(perception): add new object detection model

Implement YOLOv8-based object detection with TensorRT optimization.
Supports both camera and LiDAR fusion modes.

Closes #123
```

```
fix(planning): correct trajectory interpolation at low speeds

Previous implementation caused vehicle jitter below 1 m/s.
Added minimum velocity threshold and smooth interpolation.
```

### Pull Request Process

1. **Create feature branch** from main
2. **Make changes** following code standards
3. **Run pre-commit**: `pre-commit run --all-files`
4. **Commit** with conventional commit format
5. **Push** to remote
6. **Create PR** with template:
   - Description of changes
   - Testing performed
   - Related issues
7. **Pass CI checks**:
   - Pre-commit validation
   - Semantic PR title check
   - DCO (Developer Certificate of Origin)
8. **Code review** by maintainers (see `.github/CODEOWNERS`)
9. **Merge** when approved and CI passes

### Developer Certificate of Origin (DCO)

All commits MUST be signed off:

```bash
git commit -s -m "feat: your commit message"
```

This adds: `Signed-off-by: Your Name <your.email@example.com>`

## Build System

### colcon (ROS 2 Build Tool)

**Basic commands**:
```bash
# Full build
colcon build --symlink-install

# Build specific packages
colcon build --packages-select package_name

# Build with release optimizations
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install

# Test
colcon test
colcon test-result --verbose
```

### Build Optimization

**ccache** (compiler cache) is automatically used for faster rebuilds:

```bash
# Check ccache statistics
ccache -s

# Clear cache if needed
ccache -C
```

**Memory-constrained builds**:
```bash
# Use sequential executor (prevents parallel build spikes)
colcon build --executor sequential

# Limit parallel jobs
colcon build --parallel-workers 4
```

### Environment Sourcing

**Always source** before running ROS commands:

```bash
# Source ROS installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Source workspace
source install/setup.bash

# Or combined in ~/.bashrc:
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/autoware.shuttle_bus/install/setup.bash" >> ~/.bashrc
```

## Docker Workflow

### Multi-Stage Build System

The repository uses a sophisticated multi-stage Dockerfile for component-based builds:

**Build stages**:
- `core-devel` / `core` - Core packages
- `universe-sensing-perception-devel` / `universe-sensing-perception` - Sensing/perception
- `universe-localization-mapping-devel` / `universe-localization-mapping` - Localization/mapping
- `universe-planning-control-devel` / `universe-planning-control` - Planning/control
- `universe-vehicle-system-devel` / `universe-vehicle-system` - Vehicle interface
- `universe-devel` / `universe` - All-in-one
- Add `-cuda` suffix for GPU variants (e.g., `universe-devel-cuda`)

### Building Docker Images

```bash
# Build locally (using docker/build.sh)
cd docker
./build.sh --platform amd64  # or arm64
./build.sh --platform amd64 --cuda  # with CUDA support

# Build specific target
docker build -t autoware:universe-devel --target universe-devel .
```

### Running Containers

**Docker Compose** (recommended):
```bash
# Start all services
docker compose up

# GPU support
docker compose -f docker-compose.gpu.yaml up

# Specific services
docker compose up planning control
```

**Manual run**:
```bash
# Interactive shell in development container
docker run -it --rm \
  --network host \
  -v $(pwd):/workspace \
  ghcr.io/autowarefoundation/autoware:universe-devel \
  bash

# With NVIDIA GPU
docker run -it --rm \
  --gpus all \
  --network host \
  -v $(pwd):/workspace \
  ghcr.io/autowarefoundation/autoware:universe-devel-cuda \
  bash
```

### Container Registry

Pre-built images available at `ghcr.io/autowarefoundation/autoware:*`:
- `universe-devel` - Latest development environment
- `universe-devel-cuda` - With CUDA/TensorRT
- `universe` - Runtime only
- Tags: `latest`, `humble`, `jazzy`, version numbers

## CI/CD Pipeline

### GitHub Actions Workflows

**On Pull Requests**:
- `pre-commit.yaml` - Code quality checks (BLOCKING)
- `semantic-pull-request.yaml` - Commit message validation (BLOCKING)
- `dco.yaml` - Developer Certificate of Origin check (BLOCKING)

**Scheduled (Daily at 12:00 UTC)**:
- `health-check.yaml` - Build Docker images for amd64/arm64
- `scenario-test.yaml` - End-to-end integration testing

**On Push to Main**:
- `docker-build-and-push.yaml` - Multi-platform image builds
- Version bump workflows for .repos files

**Quarterly**:
- `pre-commit-autoupdate.yaml` - Update pre-commit hook versions

### Manual CI Triggers

Add labels to PR to trigger workflows:
- `run:health-check` - Run Docker health check workflow
- `run:scenario-test` - Run scenario testing workflow

### Web.Auto CI Configuration

`.webauto-ci.yaml` defines custom CI phases:

1. **environment-setup** - Base system (cached, rarely changes)
2. **autoware-setup** - Ansible environment (cached)
3. **autoware-build** - colcon build with ccache (incremental)

**Build optimization**: 32 parallel workers, ccache caching

## Testing Strategy

### Unit Testing

**C++ (GTest)**:
```cpp
// test/test_object_detector.cpp
#include <gtest/gtest.h>
#include "my_package/object_detector.hpp"

TEST(ObjectDetectorTest, InitializationTest) {
  rclcpp::NodeOptions options;
  auto detector = std::make_shared<ObjectDetector>(options);
  ASSERT_NE(detector, nullptr);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

**Python (pytest)**:
```python
# test/test_object_processor.py
import pytest
from my_package.object_processor import ObjectProcessor

def test_initialization():
    processor = ObjectProcessor()
    assert processor.threshold == 0.5

def test_object_filtering():
    processor = ObjectProcessor()
    # Test logic
```

**Run tests**:
```bash
# Build and test
colcon build --packages-select my_package
colcon test --packages-select my_package

# View results
colcon test-result --verbose

# With coverage
colcon test --mixin coverage
```

### Integration Testing

**Scenario Testing** (via GitHub Actions):
- Uses Scenario Simulator v2
- Executes realistic autonomous driving scenarios
- Validates end-to-end system behavior
- Runs daily at 12:00 UTC

**Manual scenario testing**:
```bash
# Download sample scenario and map
# (URLs in .github/workflows/scenario-test.yaml)

# Run scenario test
ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  scenario:=path/to/scenario.yaml \
  sensor_model:=sample_sensor_kit \
  vehicle_model:=sample_vehicle
```

## Package Development

### Creating a New ROS 2 Package

```bash
# Navigate to appropriate directory in src/
cd src/universe/autoware_universe/perception/

# Create package
ros2 pkg create my_object_detector \
  --build-type ament_cmake \
  --dependencies rclcpp autoware_msgs sensor_msgs \
  --node-name my_object_detector_node

# Package structure:
# my_object_detector/
# ├── CMakeLists.txt
# ├── package.xml
# ├── include/my_object_detector/
# ├── src/
# ├── launch/
# ├── config/
# └── test/
```

### CMakeLists.txt Template

```cmake
cmake_minimum_required(VERSION 3.14)
project(my_object_detector)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/object_detector_node.cpp
  src/object_detector_core.cpp
)

# Executable
ament_auto_add_executable(${PROJECT_NAME}_node
  src/object_detector_node.cpp
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_${PROJECT_NAME}
    test/test_object_detector.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_object_detector</name>
  <version>1.0.0</version>
  <description>Object detection node for Autoware</description>

  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake_auto</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclcpp_components</depend>
  <depend>autoware_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <test_depend>ament_cmake_gtest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Launch File Template

```xml
<launch>
  <arg name="input_topic" default="/sensing/lidar/points"/>
  <arg name="output_topic" default="/perception/objects"/>
  <arg name="detection_threshold" default="0.5"/>

  <node pkg="my_object_detector" exec="my_object_detector_node" name="my_object_detector">
    <param name="input_topic" value="$(var input_topic)"/>
    <param name="output_topic" value="$(var output_topic)"/>
    <param name="detection_threshold" value="$(var detection_threshold)"/>
    <param from="$(find-pkg-share my_object_detector)/config/default.param.yaml"/>
  </node>
</launch>
```

### Parameter File Template

```yaml
# config/default.param.yaml
/**:
  ros__parameters:
    detection_threshold: 0.5
    max_detection_range: 100.0
    min_object_size: 0.3
    use_gpu: true
    model_path: "$(env HOME)/autoware_data/models/detector.onnx"
```

## Common Tasks and Solutions

### Task: Update Autoware Dependencies

```bash
# 1. Check for updates in .repos files
git pull origin main

# 2. Update source repositories
vcs import src < autoware.repos --force
vcs pull src

# 3. Update rosdep
rosdep update
rosdep install -y --from-paths src --ignore-src

# 4. Rebuild
colcon build --symlink-install
```

### Task: Add a New Dependency Repository

Edit `autoware.repos`:

```yaml
repositories:
  universe/external/my_new_package:
    type: git
    url: https://github.com/organization/my_new_package.git
    version: main  # or specific tag/commit
```

Then import:
```bash
vcs import src < autoware.repos
```

### Task: Debug Build Failures

```bash
# 1. Clean build artifacts
rm -rf build/ install/ log/

# 2. Build with verbose output
colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON

# 3. Build single package with debug symbols
colcon build --packages-select my_package \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 4. Check compilation database
cat build/my_package/compile_commands.json
```

### Task: Fix Pre-commit Hook Failures

```bash
# 1. Run hooks manually to see failures
pre-commit run --all-files

# 2. Auto-fix formatting issues
pre-commit run --all-files

# 3. If clang-format fails, apply formatting
find src/my_package -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

# 4. If Python formatting fails
black src/my_package
isort src/my_package

# 5. Skip hooks temporarily (NOT RECOMMENDED, only for emergency)
git commit --no-verify -m "..."
```

### Task: Run Specific Component

```bash
# Source workspace
source install/setup.bash

# Launch specific component
ros2 launch autoware_launch planning_simulator.launch.xml \
  vehicle_model:=sample_vehicle \
  sensor_model:=sample_sensor_kit \
  map_path:=$HOME/autoware_map/sample-map-planning

# Check running nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /perception/object_recognition/objects

# Check parameters
ros2 param list /my_node
ros2 param get /my_node my_parameter
```

### Task: Profile Performance

```bash
# CPU profiling with perf
perf record -g ros2 launch ...
perf report

# Memory profiling with valgrind
valgrind --tool=massif ros2 run my_package my_node

# ROS 2 performance testing
ros2 run performance_test perf_test \
  --topics /perception/objects \
  --rate 10

# Check computational graph
ros2 run rqt_graph rqt_graph
```

### Task: Debugging with GDB

```bash
# Attach to running node
ros2 run --prefix 'gdb -ex run --args' my_package my_node

# Or debug in VS Code with launch.json:
{
  "type": "cppdbg",
  "request": "launch",
  "name": "Debug my_node",
  "program": "${workspaceFolder}/install/my_package/lib/my_package/my_node",
  "args": [],
  "stopAtEntry": false,
  "cwd": "${workspaceFolder}",
  "environment": [],
  "sourceFileMap": {
    "/workspace": "${workspaceFolder}"
  }
}
```

## Troubleshooting

### Build Errors

**Error: Package not found**
```
CMake Error at CMakeLists.txt:10 (find_package):
  By not providing "FindXXX.cmake"...
```
**Solution**: Install missing dependency
```bash
rosdep install --from-paths src --ignore-src -y
```

**Error: Out of memory during build**
**Solution**: Use sequential executor or limit workers
```bash
colcon build --executor sequential
# or
colcon build --parallel-workers 2
```

**Error: ccache: error: Failed to create directory**
**Solution**: Clear ccache or set cache directory
```bash
ccache -C  # Clear cache
export CCACHE_DIR=$HOME/.ccache
```

### Runtime Errors

**Error: Package 'my_package' not found**
**Solution**: Rebuild and source workspace
```bash
colcon build --packages-select my_package
source install/setup.bash
```

**Error: [WARN] [rmw_cyclonedds_cpp]: Unable to connect to publisher**
**Solution**: Check RMW configuration and network
```bash
# Check RMW implementation
echo $RMW_IMPLEMENTATION  # Should be rmw_cyclonedds_cpp

# Check DDS discovery
ros2 daemon stop
ros2 daemon start
ros2 topic list
```

**Error: Transform lookup would require extrapolation into the past**
**Solution**: Check TF2 tree and synchronization
```bash
ros2 run tf2_tools view_frames
# Examine generated frames.pdf for missing transforms
```

### Docker Issues

**Error: docker: Error response from daemon: could not select device driver**
**Solution**: Install NVIDIA Container Toolkit
```bash
./setup-dev-env.sh --no-cuda  # If no GPU available
# or
sudo apt-get install nvidia-container-toolkit
sudo systemctl restart docker
```

**Error: Permission denied while connecting to Docker daemon**
**Solution**: Add user to docker group
```bash
sudo usermod -aG docker $USER
newgrp docker
```

## Key File Reference

### Configuration Files

| File | Purpose | Location |
|------|---------|----------|
| `.pre-commit-config.yaml` | Code quality hooks | `/home/user/autoware.shuttle_bus/.pre-commit-config.yaml` |
| `.clang-format` | C++ formatting rules | `/home/user/autoware.shuttle_bus/.clang-format` |
| `.clang-tidy` | C++ static analysis | `/home/user/autoware.shuttle_bus/.clang-tidy` |
| `CPPLINT.cfg` | cpplint configuration | `/home/user/autoware.shuttle_bus/CPPLINT.cfg` |
| `setup.cfg` | Python tools (flake8, isort) | `/home/user/autoware.shuttle_bus/setup.cfg` |
| `.yamllint.yaml` | YAML linting | `/home/user/autoware.shuttle_bus/.yamllint.yaml` |
| `.prettierrc.yaml` | Prettier formatting | `/home/user/autoware.shuttle_bus/.prettierrc.yaml` |
| `.markdownlint.yaml` | Markdown linting | `/home/user/autoware.shuttle_bus/.markdownlint.yaml` |
| `.hadolint.yaml` | Dockerfile linting | `/home/user/autoware.shuttle_bus/.hadolint.yaml` |

### Build and Dependency Files

| File | Purpose | Location |
|------|---------|----------|
| `autoware.repos` | Main dependencies | `/home/user/autoware.shuttle_bus/autoware.repos` |
| `simulator.repos` | Simulator dependencies | `/home/user/autoware.shuttle_bus/simulator.repos` |
| `tools.repos` | Development tools | `/home/user/autoware.shuttle_bus/tools.repos` |
| `setup-dev-env.sh` | Environment setup script | `/home/user/autoware.shuttle_bus/setup-dev-env.sh` |
| `amd64.env` | Build environment variables | `/home/user/autoware.shuttle_bus/amd64.env` |
| `amd64_jazzy.env` | Jazzy-specific overrides | `/home/user/autoware.shuttle_bus/amd64_jazzy.env` |
| `arm64.env` | ARM64 configuration | `/home/user/autoware.shuttle_bus/arm64.env` |

### Docker Files

| File | Purpose | Location |
|------|---------|----------|
| `docker/Dockerfile` | Multi-stage build | `/home/user/autoware.shuttle_bus/docker/Dockerfile` |
| `docker/docker-compose.yaml` | Service orchestration | `/home/user/autoware.shuttle_bus/docker/docker-compose.yaml` |
| `docker/docker-compose.gpu.yaml` | GPU variant | `/home/user/autoware.shuttle_bus/docker/docker-compose.gpu.yaml` |
| `docker/build.sh` | Build helper script | `/home/user/autoware.shuttle_bus/docker/build.sh` |

### CI/CD Files

| File | Purpose | Location |
|------|---------|----------|
| `.github/workflows/` | GitHub Actions workflows | `/home/user/autoware.shuttle_bus/.github/workflows/` |
| `.webauto-ci.yaml` | Web.Auto CI configuration | `/home/user/autoware.shuttle_bus/.webauto-ci.yaml` |
| `.github/CODEOWNERS` | Code ownership | `/home/user/autoware.shuttle_bus/.github/CODEOWNERS` |

## Important Notes for AI Assistants

### When Making Code Changes

1. **ALWAYS run pre-commit before committing**
   - Many formatting issues are auto-fixed by hooks
   - Don't waste time manually formatting if hooks will fix it

2. **Check if files are in `src/`**
   - Files in `src/` come from external repos (via .repos files)
   - Don't edit them here; propose changes to upstream repositories
   - This repository only contains configuration and orchestration

3. **Follow Conventional Commits**
   - PR will be blocked if commit messages don't follow format
   - Use `feat:`, `fix:`, `docs:`, etc.

4. **Understand the component architecture**
   - Changes to perception go in `autoware_universe/perception/`
   - Changes to planning go in `autoware_universe/planning/`
   - Launch configuration changes go in `autoware_launch/` (custom fork)

5. **Test your changes**
   - Build: `colcon build --packages-select my_package`
   - Test: `colcon test --packages-select my_package`
   - Integration test if possible

6. **Use appropriate issue labels and PR templates**
   - Follow PR template in `.github/pull_request_template.md`
   - Add appropriate labels for maintainer routing

### When Debugging Issues

1. **Check the layer where issue occurs**
   - Build-time: Check CMakeLists.txt, package.xml, dependencies
   - Run-time: Check launch files, parameters, TF2 tree
   - Communication: Check topics, DDS configuration

2. **Use ROS 2 introspection tools**
   - `ros2 node list` - See running nodes
   - `ros2 topic echo` - Monitor messages
   - `ros2 param get` - Check parameters
   - `ros2 run tf2_tools view_frames` - Visualize transforms

3. **Check logs in detail**
   - Build logs: `log/latest_build/`
   - Runtime logs: `~/.ros/log/`
   - Docker logs: `docker logs <container>`

4. **Consult Autoware documentation**
   - Main docs: https://autowarefoundation.github.io/autoware-documentation/
   - Component-specific READMEs in `src/universe/autoware_universe/`

### When Proposing Changes

1. **Small, focused PRs are better**
   - One logical change per PR
   - Easier to review and merge
   - Reduces merge conflicts

2. **Document WHY, not just WHAT**
   - Commit messages should explain motivation
   - Code comments explain complex algorithms
   - README updates for new features

3. **Consider backward compatibility**
   - Autoware is used in production vehicles
   - Breaking changes need discussion and migration path
   - Document deprecations clearly

4. **Performance matters**
   - Autonomous driving is real-time critical
   - Profile before optimizing
   - Document performance characteristics

## Additional Resources

- **Autoware Documentation**: https://autowarefoundation.github.io/autoware-documentation/
- **Autoware Foundation**: https://www.autoware.org/
- **GitHub Organization**: https://github.com/autowarefoundation
- **Discord Community**: https://discord.gg/Q94UsPvReQ
- **Support Guidelines**: https://autowarefoundation.github.io/autoware-documentation/main/support/support-guidelines/

## Version Information

- **Document Version**: 1.0.0
- **Last Updated**: 2025-11-16
- **Primary ROS Distro**: Humble
- **Autoware Core Version**: 1.4.0
- **Autoware Universe Version**: 0.47.1
- **Autoware Launch**: 0.47.0 (custom fork: HTOLCH/nuway_autoware_launch)

---

**Note**: This document is maintained for AI assistants working with this codebase. Keep it updated as the project evolves. For human documentation, refer to the official Autoware documentation at https://autowarefoundation.github.io/autoware-documentation/.
