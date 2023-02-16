#!/bin/sh

# Server that waits for scenario and map upload
python3 upload_server.py
echo "Upload done, starting scenariorunner with carla host $1"

# Start of scenariorunner, host is the carla instance (see dockerfile)
python3 scenario_runner.py --sync --host $1
