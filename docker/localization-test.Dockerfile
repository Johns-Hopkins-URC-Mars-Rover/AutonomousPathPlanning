# Compatibility alias: build the Gazebo/Humble image first with
# scripts/run_simulation_container.sh --build true
# Tests use the active rover_localization package and upstream Humble binaries.
FROM rover-gazebo:humble
CMD ["bash", "/source/docker/run-localization-test.sh"]
