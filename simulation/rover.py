
import math

from navigation.heading import normalize_angle
from navigation.home_vector import HomeVector
from safety.governor import MotionCommand


class Rover:
    def __init__(self, x=0.0, y=0.0, heading=0.0):
        self.x = x
        self.y = y
        self.heading = heading

        self.home_vector = HomeVector()
        self.mode = "EXPLORE"

        # Obstacle avoidance state.
        self.avoiding_obstacle = False
        self.homing_detour = False
        self.turn_direction = 1.0
        self.avoidance_steps = 0
        self.clear_steps = 0

        self.obstacle_threshold = 0.8
        self.clearance_threshold = 1.2
        self.turn_rate = 1.5
        self.heading_tolerance = 0.12
        self.home_radius = 0.05

        self.max_avoidance_steps = 50
        self.avoidance_turn_steps = 8
        self.avoidance_forward_speed = 0.25
        self.avoidance_forward_threshold = 0.75
        self.required_clear_steps = 5

        # Recovery state.
        self.recovering = False
        self.recovery_steps = 0
        self.max_recovery_steps = 30
        self.recovery_clearance = 1.2
        self.recovery_turn_steps = 0
        self.max_recovery_turn_steps = 24
        self.recovery_forward_steps = 0
        self.max_recovery_forward_steps = 5

        # Progress tracking.
        self.last_position = (x, y)
        self.no_progress_steps = 0
        self.progress_epsilon = 0.001

        # Near-home approach settings.
        self.home_approach_radius = 0.6
        self.home_approach_threshold = 0.25

    def return_home(self):
        self.mode = "RETURN_HOME"

    def _distance_at_heading(self, world, heading):
        obstacle = world.front_obstacle(
            self.x,
            self.y,
            heading,
        )

        if obstacle is None:
            return world.max_sensor_range

        return obstacle["distance"]

    def _choose_turn_direction(self, world):
        left_heading = normalize_angle(
            self.heading + math.radians(45)
        )
        right_heading = normalize_angle(
            self.heading - math.radians(45)
        )

        left_clearance = self._distance_at_heading(
            world, left_heading
        )
        right_clearance = self._distance_at_heading(
            world, right_heading
        )

        if left_clearance > right_clearance:
            return 1.0

        if right_clearance > left_clearance:
            return -1.0

        # Keep the previous direction if both sides are equally clear.
        return self.turn_direction

    def _home_heading(self):
        return math.atan2(-self.y, -self.x)

    def _clear_toward_home(self, world):
        distance = self._distance_at_heading(
            world, self._home_heading()
        )
        return distance >= self.home_approach_threshold

    def _home_route_clear(self, world):
        """Check whether the direct homeward route has enough clearance."""
        distance = self._distance_at_heading(
            world, self._home_heading()
        )
        return distance >= self.clearance_threshold

    def _integrate(self, command, dt):
        self.heading = normalize_angle(
            self.heading + command.turn_rate * dt
        )

        self.x += command.speed * math.cos(self.heading) * dt
        self.y += command.speed * math.sin(self.heading) * dt

        self.home_vector.update(
            self.heading,
            command.speed,
            dt,
        )

    def _update_progress(self):
        current_position = (self.x, self.y)

        movement = math.hypot(
            current_position[0] - self.last_position[0],
            current_position[1] - self.last_position[1],
        )

        if movement < self.progress_epsilon:
            self.no_progress_steps += 1
        else:
            self.no_progress_steps = 0

        self.last_position = current_position

    def _execute(self, command, dt, obstacle):
        self._integrate(command, dt)
        self._update_progress()
        return obstacle, command

    def _start_recovery(self, world):
        self.avoiding_obstacle = False
        self.avoidance_steps = 0
        self.clear_steps = 0

        self.recovering = True
        self.recovery_steps = 0
        self.recovery_turn_steps = 0
        self.recovery_forward_steps = 0

        self.turn_direction = self._choose_turn_direction(world)

    def _finish_recovery(self):
        self.recovering = False
        self.recovery_steps = 0
        self.recovery_turn_steps = 0
        self.recovery_forward_steps = 0

        self.avoiding_obstacle = False
        self.avoidance_steps = 0
        self.clear_steps = 0

    def step(self, world, dt=0.1):
        if dt <= 0:
            raise ValueError("dt must be positive")

        distance_home = math.hypot(self.x, self.y)

        # Stop after reaching home.
        if (
            self.mode == "RETURN_HOME"
            and distance_home <= self.home_radius
        ):
            self._finish_recovery()
            self.homing_detour = False
            return None, MotionCommand(0.0, 0.0)

        obstacle = world.front_obstacle(
            self.x,
            self.y,
            self.heading,
        )

        obstacle_distance = (
            obstacle["distance"]
            if obstacle is not None
            else world.max_sensor_range
        )

        # --------------------------------------------------
        # Recovery state machine.
        # --------------------------------------------------
        if self.recovering:
            self.recovery_steps += 1

            # Advance only when there is adequate clearance.
            if obstacle_distance >= self.recovery_clearance:
                self.recovery_forward_steps += 1

                command = MotionCommand(
                    self.avoidance_forward_speed,
                    0.0,
                )

                if (
                    self.recovery_forward_steps
                    >= self.max_recovery_forward_steps
                ):
                    self._finish_recovery()

                return self._execute(command, dt, obstacle)

            self.recovery_forward_steps = 0
            self.recovery_turn_steps += 1

            # Commit to a turn rather than reversing direction
            # whenever the sensor reading changes.
            if (
                self.recovery_turn_steps
                >= self.max_recovery_turn_steps
            ):
                self.turn_direction = self._choose_turn_direction(
                    world
                )
                self.recovery_turn_steps = 0

            command = MotionCommand(
                0.0,
                self.turn_direction * self.turn_rate,
            )
            return self._execute(command, dt, obstacle)

        # --------------------------------------------------
        # Near-home direct approach.
        # Do not bypass an active homing detour.
        # --------------------------------------------------
        near_home = (
            self.mode == "RETURN_HOME"
            and distance_home <= self.home_approach_radius
        )

        if (
            near_home
            and self._clear_toward_home(world)
            and not self.homing_detour
        ):
            self.avoiding_obstacle = False
            self.avoidance_steps = 0
            self.clear_steps = 0

            target_heading = self._home_heading()
            heading_error = normalize_angle(
                target_heading - self.heading
            )

            if abs(heading_error) > self.heading_tolerance:
                command = MotionCommand(
                    0.0,
                    math.copysign(
                        self.turn_rate,
                        heading_error,
                    ),
                )
            else:
                command = MotionCommand(0.35, 0.0)

            return self._execute(command, dt, obstacle)

        # --------------------------------------------------
        # Start obstacle avoidance.
        # --------------------------------------------------
        if (
            not self.avoiding_obstacle
            and obstacle_distance < self.obstacle_threshold
        ):
            self.avoiding_obstacle = True
            self.avoidance_steps = 0
            self.clear_steps = 0

            # Preserve the detour across avoidance exit.
            if self.mode == "RETURN_HOME":
                self.homing_detour = True

            self.turn_direction = self._choose_turn_direction(
                world
            )

        # --------------------------------------------------
        # Active obstacle avoidance with clearance hysteresis.
        # --------------------------------------------------
        if self.avoiding_obstacle:
            self.avoidance_steps += 1

            # Avoidance has persisted too long: enter recovery.
            if self.avoidance_steps >= self.max_avoidance_steps:
                self._start_recovery(world)

                command = MotionCommand(
                    0.0,
                    self.turn_direction * self.turn_rate,
                )
                return self._execute(command, dt, obstacle)

            if obstacle_distance >= self.clearance_threshold:
                self.clear_steps += 1

                # Keep moving cautiously through the clear area.
                command = MotionCommand(
                    self.avoidance_forward_speed,
                    0.0,
                )

                if self.clear_steps >= self.required_clear_steps:
                    self.avoiding_obstacle = False
                    self.avoidance_steps = 0
                    self.clear_steps = 0

                return self._execute(command, dt, obstacle)

            # The path is not consistently clear.
            self.clear_steps = 0

            if self.avoidance_steps <= self.avoidance_turn_steps:
                command = MotionCommand(
                    0.0,
                    self.turn_direction * self.turn_rate,
                )

            elif (
                obstacle_distance
                >= self.avoidance_forward_threshold
            ):
                command = MotionCommand(
                    self.avoidance_forward_speed,
                    0.0,
                )

            else:
                command = MotionCommand(
                    0.0,
                    self.turn_direction * self.turn_rate,
                )

            return self._execute(command, dt, obstacle)

        # --------------------------------------------------
        # Persistent homing detour.
        # Do not resume direct homing until the homeward
        # sensor direction has sufficient clearance.
        # --------------------------------------------------
        if self.homing_detour and self.mode == "RETURN_HOME":
            if self._home_route_clear(world):
                self.homing_detour = False
            else:
                if (
                    obstacle_distance
                    >= self.avoidance_forward_threshold
                ):
                    command = MotionCommand(
                        self.avoidance_forward_speed,
                        0.0,
                    )
                else:
                    command = MotionCommand(
                        0.0,
                        self.turn_direction * self.turn_rate,
                    )

                return self._execute(command, dt, obstacle)

        # --------------------------------------------------
        # Normal navigation.
        # --------------------------------------------------
        if self.mode == "RETURN_HOME":
            target_heading = self._home_heading()
            heading_error = normalize_angle(
                target_heading - self.heading
            )

            if abs(heading_error) > self.heading_tolerance:
                command = MotionCommand(
                    0.0,
                    math.copysign(
                        self.turn_rate,
                        heading_error,
                    ),
                )
            else:
                command = MotionCommand(0.7, 0.0)

        else:
            command = MotionCommand(1.0, 0.0)

        return self._execute(command, dt, obstacle)
