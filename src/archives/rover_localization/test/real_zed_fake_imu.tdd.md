# Real ZED with fake IMU: TDD evidence

No source plan was supplied. The behavior was derived from the requested bench
test: use real ZED optical odometry while a fixture supplies the only
`/imu/fused` stream.

| Guarantee | Test | RED | GREEN |
| --- | --- | --- | --- |
| The bench launch includes the real-ZED localization launch, starts the fake IMU executable, and defaults to `/imu/fused`. | `ruby src/rover_localization/test/test_localization_contract.rb` | 5 runs, 25 assertions, 1 error because `zed_fake_imu_localization.launch.py` was absent. | 5 runs, 33 assertions, 0 failures/errors/skips. |
| Launch and publisher Python source parse. | `python3 -m py_compile src/rover_localization/launch/*.launch.py src/rover_localization/scripts/fake_imu_publisher.py` | Not applicable: the files did not exist. | Completed successfully. |
| Existing simulation-output evaluation stays valid. | `python3 -m unittest discover -s src/rover_localization/test -p test_smoke_probe.py -v` | Existing suite was green before this feature. | 5 tests passed. |

Coverage was not measured. ROS 2 Humble and `colcon` are unavailable in this
macOS checkout, so the real camera, ROS graph, CMake package installation, and
publisher rate still require the documented Ubuntu/ROS runtime bench test.
