
import math

from simulation.evaluate import generate_extra_scenarios
from simulation.rover import Rover
from simulation.world import World


SEEDS = [7, 21, 42, 73, 101, 123, 256, 512, 999, 2026]
SCENARIOS_PER_SEED = 20
RETURN_STEP = 150
MAX_STEPS = 750
DT = 0.1
HOME_RADIUS = 0.05

FOCUS_FAILURES = {
    (2026, "Generated 07"),
}

# Print regular snapshots and every state transition or stationary step.
PRINT_EVERY = 25
STATIONARY_EPSILON = 1e-9


def run_diagnostic(seed, name, obstacles):
    world = World(obstacles=obstacles)
    rover = Rover()

    arrival_step = None
    previous_position = (rover.x, rover.y)
    previous_avoiding = rover.avoiding_obstacle
    previous_recovering = getattr(rover, "recovering", False)
    previous_homing_detour = getattr(rover, "homing_detour", False)

    print(f"\n{'=' * 72}")
    print(f"Seed {seed} / {name}")
    print(f"Obstacles: {obstacles}")
    print(f"{'=' * 72}")

    for step in range(MAX_STEPS):
        if step == RETURN_STEP:
            rover.return_home()
            print(
                f"\n--- RETURN HOME STARTS "
                f"(step {step + 1}) ---"
            )

            # Reset snapshots to the actual state at return-home start.
            previous_position = (rover.x, rover.y)
            previous_avoiding = rover.avoiding_obstacle
            previous_recovering = getattr(
                rover, "recovering", False
            )
            previous_homing_detour = getattr(
                rover, "homing_detour", False
            )

        obstacle, command = rover.step(world, dt=DT)

        current_position = (rover.x, rover.y)
        movement = math.hypot(
            current_position[0] - previous_position[0],
            current_position[1] - previous_position[1],
        )

        recovering = getattr(rover, "recovering", False)
        recovery_steps = getattr(rover, "recovery_steps", 0)
        homing_detour = getattr(rover, "homing_detour", False)

        state_changed = (
            rover.avoiding_obstacle != previous_avoiding
            or recovering != previous_recovering
            or homing_detour != previous_homing_detour
        )

        stationary = movement < STATIONARY_EPSILON

        # Detailed logging during the return phase only.
        if step >= RETURN_STEP:
            relative_step = step - RETURN_STEP + 1

            should_print = (
                relative_step <= 20
                or relative_step % PRINT_EVERY == 0
                or stationary
                or state_changed
            )

            if should_print:
                obstacle_distance = (
                    obstacle["distance"]
                    if obstacle is not None
                    else world.max_sensor_range
                )

                print(
                    f"step={step + 1:3d} "
                    f"pos=({rover.x:.3f}, {rover.y:.3f}) "
                    f"home={math.hypot(rover.x, rover.y):.3f} "
                    f"heading={rover.heading:.3f} "
                    f"speed={command.speed:.2f} "
                    f"turn={command.turn_rate:.2f} "
                    f"obstacle={obstacle_distance:.3f} "
                    f"avoiding={rover.avoiding_obstacle} "
                    f"avoid_steps={rover.avoidance_steps} "
                    f"homing_detour={homing_detour} "
                    f"recovering={recovering} "
                    f"recovery_steps={recovery_steps} "
                    f"moved={movement:.4f}"
                )

        if (
            step >= RETURN_STEP
            and math.hypot(rover.x, rover.y) <= HOME_RADIUS
        ):
            arrival_step = step + 1
            print(f"\nHOME REACHED at step {arrival_step}")
            break

        previous_position = current_position
        previous_avoiding = rover.avoiding_obstacle
        previous_recovering = recovering
        previous_homing_detour = homing_detour

    final_distance = math.hypot(rover.x, rover.y)

    print("\n--- FINAL RESULT ---")
    print(f"Seed: {seed}")
    print(f"Scenario: {name}")
    print(f"Arrived: {arrival_step is not None}")
    print(f"Arrival step: {arrival_step}")
    print(f"Final distance: {final_distance:.3f} m")
    print(f"Final position: ({rover.x:.3f}, {rover.y:.3f})")
    print(f"Final heading: {rover.heading:.3f} rad")
    print(f"Final avoiding: {rover.avoiding_obstacle}")
    print(
        f"Final homing detour: "
        f"{getattr(rover, 'homing_detour', False)}"
    )
    print(f"Final recovery: {getattr(rover, 'recovering', False)}")
    print(f"No-progress steps: {rover.no_progress_steps}")


def main():
    print("=== FOCUSED FAILURE DIAGNOSTICS ===")
    print(f"Return starts at simulation step {RETURN_STEP + 1}")
    print(f"Maximum simulation steps: {MAX_STEPS}")

    for seed in SEEDS:
        scenarios = generate_extra_scenarios(
            count=SCENARIOS_PER_SEED,
            seed=seed,
        )

        for name, obstacles in scenarios.items():
            if (seed, name) in FOCUS_FAILURES:
                run_diagnostic(seed, name, obstacles)


if __name__ == "__main__":
    main()
