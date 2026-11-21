# SPDX-FileCopyrightText: 2025 Anthony Tracy
# SPDX-License-Identifier: MIT
#
# Shortcuts for the Docker commands used day to day. `make` lists them.
# Everything runs in containers; the host only needs Docker (and X11 for the
# GUI targets).

COMPOSE := docker compose -f docker/docker-compose.yml
SIM     := rc-racer-sim-manual
DEV_IMG := rc-racer-dev:latest
WORLD   ?= practice_course.world
PKG     ?=
FILTER  ?=

# Run a ROS command inside the running sim-manual container
ROS = docker exec -it $(SIM) bash -c "source install/setup.bash && $(1)"

.DEFAULT_GOAL := help
.PHONY: help images sim down shell keyboard joy-echo joy-devices joy-calibrate \
        joy-sdl-map se-reset se-diagnostics se-watch se-deactivate se-activate test test-system \
        _sim-running

help: ## List the targets
	@grep -E '^[a-z][a-z-]+:.*## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'
	@echo
	@echo "  Variables: WORLD=<file>.world (sim), PKG=<package> (test), FILTER=<regex> (test-system)"

# --- Images ------------------------------------------------------------------

images: ## Build the simulation and dev images (after Dockerfile changes)
	$(COMPOSE) build simulation dev

# --- Driving the simulation by hand ------------------------------------------

sim: ## Gazebo GUI + state_estimation + RViz + Xbox teleop (Ctrl+C stops)
	@xhost +local:docker >/dev/null 2>&1 || echo "xhost not found: GUI windows may not open"
	SIM_WORLD=$(WORLD) $(COMPOSE) up sim-manual

down: ## Stop and remove all project containers
	$(COMPOSE) down

shell: _sim-running ## Shell in the running sim container (ROS sourced)
	docker exec -it $(SIM) bash -c "source install/setup.bash && exec bash"

keyboard: _sim-running ## Drive with the keyboard instead of the controller
	$(call ROS,ros2 run teleop_twist_keyboard teleop_twist_keyboard)

# --- Game controller -----------------------------------------------------------

joy-echo: _sim-running ## Show what the controller sends on /joy
	$(call ROS,ros2 topic echo /joy)

joy-devices: _sim-running ## List controllers SDL can see, with their GUIDs
	$(call ROS,ros2 run joy joy_enumerate_devices)

joy-calibrate: _sim-running ## Detect the pad's buttons/axes and apply them to teleop (until restart)
	$(call ROS,ros2 run rc_racer_bringup joy_calibrate.py)

joy-sdl-map: _sim-running ## Measure the raw pad and print an SDL mapping for docker-compose.yml
	$(call ROS,ros2 run rc_racer_bringup joy_calibrate.py --sdl-map)

# --- state_estimation in the running sim -------------------------------------

se-reset: _sim-running ## Reset the estimate to the odom origin (~/reset)
	$(call ROS,ros2 service call /state_estimation/reset std_srvs/srv/Trigger)

se-diagnostics: _sim-running ## Show state_estimation's latest health report
	$(call ROS,ros2 topic echo --once /diagnostics)

se-watch: _sim-running ## Stream state_estimation's health reports, one line each (Ctrl+C stops)
	$(call ROS,ros2 run rc_racer_bringup se_watch.py)

se-deactivate: _sim-running ## Lifecycle: deactivate state_estimation (/odom stops)
	$(call ROS,ros2 lifecycle set /state_estimation deactivate)

se-activate: _sim-running ## Lifecycle: activate state_estimation (odom restarts at the car)
	$(call ROS,ros2 lifecycle set /state_estimation activate)

# --- Tests ---------------------------------------------------------------------

test: ## Build and run unit/component tests + linters in the dev image (PKG=... for one package)
	docker run --rm -v "$(CURDIR):/ws/src/rc-racer:ro" -w /ws $(DEV_IMG) bash -c '\
		source /opt/ros/humble/setup.bash && \
		colcon build $(if $(PKG),--packages-up-to $(PKG),--packages-skip rc_racer_system_tests) && \
		colcon test $(if $(PKG),--packages-select $(PKG),--packages-skip rc_racer_system_tests); \
		colcon test-result --verbose'

test-system: ## Run the Gazebo system tests, as in nightly CI (FILTER=<regex> for some)
	./scripts/run-system-tests.sh $(FILTER)

_sim-running:
	@docker ps --format '{{.Names}}' | grep -qx $(SIM) || \
		{ echo "$(SIM) is not running: start it with 'make sim' in another terminal"; exit 1; }
