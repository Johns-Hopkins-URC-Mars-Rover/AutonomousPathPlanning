# Localization verification record

The intended behavior is a configurable local EKF that accepts optical ZED odometry and one joint IMU stream, plus a hardware-free synthetic publisher that exercises the same two ROS input contracts.

| Check | RED before implementation | GREEN after implementation |
| --- | --- | --- |
| `ruby src/rover_localization/test/test_localization_contract.rb` | 2 failures and 1 error: wheel topic in EKF config; ZED override and launch installation absent | 3 tests, 23 assertions, 0 failures/errors |
| `g++ -std=c++17 -Isrc/rover_localization/include src/rover_localization/test/test_simulation_model.cpp -o /private/tmp/rover_simulation_model_test` | Compile failure: `simulation_model.hpp` absent | Compiled successfully; executable completed successfully |
| `python3 -m py_compile src/rover_localization/launch/*.launch.py` | Not applicable until launch files existed | Completed successfully |
| `git diff --check` | Not applicable | Completed successfully |

The C++ test covers straight motion, turning, and angle wrapping. The Ruby checks the YAML measurement masks, active sensor topics, ZED optical/raw-IMU settings, package dependencies, and launch-file presence. These are source and configuration checks. Coverage for the ROS node was not measured because ROS 2 and `colcon` are unavailable in this macOS checkout; Docker is installed but its daemon is not running. A ROS 2 Humble machine must still run `colcon build --packages-select rover_localization`, `colcon test --packages-select rover_localization`, and the simulated launch before the whole pipeline can be called runtime-validated.

## Header layout follow-up

The first implementation used `include/rover_localization/simulation_model.hpp`. The package-private header now lives at `include/simulation_model.hpp`, removing the nested same-name directory. The include statements and package README match the new location. Before the move, the added layout test failed and the C++ model test could not compile against the new include path. After the move, the Ruby suite passed with 4 tests and 25 assertions, and the C++ model test compiled and ran successfully. The ROS runtime gate above remains unchanged.
