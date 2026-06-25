#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# start/goal node can be overridden via env vars, e.g. for a different test map
START_NODE="${START_NODE:-S}"
GOAL_NODE="${GOAL_NODE:-T}"

# launching app: path planning (task 1) + localization (task 2) + navigation (task 3) + control (task 4)
dt-exec roslaunch --wait duckie_navigation full_system.launch \
  start:="${START_NODE}" goal:="${GOAL_NODE}"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
