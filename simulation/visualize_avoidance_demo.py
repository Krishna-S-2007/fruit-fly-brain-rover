
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from simulation.rover import Rover
from simulation.world import World


def check_collision(x, y, obstacles, rover_radius):
    """Return obstacle IDs intersecting the rover at this position."""
    collisions = []

    for index, (ox, oy, radius) in enumerate(obstacles, start=1):
        distance = math.hypot(x - ox, y - oy)

        if distance <= radius + rover_radius:
            collisions.append(index)

    return collisions


def main():
    # Keep the same three-obstacle scenario used in the successful test.
    obstacles = [
        (2.0, 0.0, 0.45),
        (4.0, 0.0, 0.50),
        (6.0, 0.0, 0.45),
    ]

    rover_radius = 0.12
    return_step = 150
    total_steps = 750
    dt = 0.1

    world = World(obstacles=obstacles)
    rover = Rover()

    # Record every position produced by the real simulation.
    trajectory_x = [rover.x]
    trajectory_y = [rover.y]

    return_position = None
    arrival_step = None
    avoidance_events = []
    collision_events = []

    previous_avoiding = rover.avoiding_obstacle
    previously_colliding = set()

    print("=== OBSTACLE-AVOIDANCE DEMONSTRATION ===")
    print(f"Obstacles: {len(obstacles)}")

    for step in range(total_steps):
        if step == return_step:
            rover.return_home()
            return_position = (rover.x, rover.y)

            print("\n--- RETURN-HOME PHASE ---")
            print(
                f"Return starts at "
                f"({rover.x:.3f}, {rover.y:.3f})"
            )

        obstacle, command = rover.step(world, dt=dt)

        trajectory_x.append(rover.x)
        trajectory_y.append(rover.y)

        # Record transitions from the actual controller state.
        avoiding = rover.avoiding_obstacle

        if avoiding != previous_avoiding:
            event = (
                step + 1,
                rover.x,
                rover.y,
                avoiding,
            )
            avoidance_events.append(event)

            label = "STARTED" if avoiding else "ENDED"
            print(
                f"Step {step + 1}: Avoidance {label} "
                f"at ({rover.x:.3f}, {rover.y:.3f})"
            )

        previous_avoiding = avoiding

        # Check the rover's physical footprint at each recorded step.
        current_collisions = set(
            check_collision(
                rover.x,
                rover.y,
                obstacles,
                rover_radius,
            )
        )

        for obstacle_id in sorted(current_collisions - previously_colliding):
            collision_events.append(
                (step + 1, obstacle_id, rover.x, rover.y)
            )
            print(
                f"Collision detected at step {step + 1}, "
                f"obstacle O{obstacle_id}"
            )

        previously_colliding = current_collisions

        if step % 25 == 0:
            obstacle_distance = (
                obstacle["distance"]
                if obstacle is not None
                else world.max_sensor_range
            )

            print(
                f"Step {step + 1:03d} | "
                f"Mode={rover.mode:11s} | "
                f"Position=({rover.x:.3f}, {rover.y:.3f}) | "
                f"Heading={rover.heading:.2f} rad | "
                f"Sensor={obstacle_distance:.2f} m | "
                f"Avoiding={avoiding}"
            )

        distance_home = math.hypot(rover.x, rover.y)

        if (
            step >= return_step
            and distance_home <= rover.home_radius
        ):
            arrival_step = step + 1
            print(f"\nHOME REACHED at step {arrival_step}")
            break

    final_distance = math.hypot(rover.x, rover.y)

    print("\n--- RESULTS ---")
    print(f"Steps simulated: {len(trajectory_x) - 1}")
    print(f"Avoidance transitions: {len(avoidance_events)}")
    print(f"Collision events: {len(collision_events)}")
    print(f"Final distance from home: {final_distance:.3f} m")
    print(f"Home reached: {arrival_step is not None}")

    # --------------------------------------------------
    # Plot the actual recorded trajectory.
    # --------------------------------------------------
    fig, ax = plt.subplots(figsize=(10, 12))

    split = min(return_step + 1, len(trajectory_x))

    # Exploration path.
    ax.plot(
        trajectory_x[:split],
        trajectory_y[:split],
        color="#2878D0",
        linewidth=2.8,
        label="Exploration",
        solid_capstyle="round",
        zorder=3,
    )

    # Return-home path.
    ax.plot(
        trajectory_x[split - 1:],
        trajectory_y[split - 1:],
        color="#F28E2B",
        linewidth=2.8,
        label="Return-home",
        solid_capstyle="round",
        zorder=4,
    )

    # Draw obstacle bodies and expanded collision envelopes.
    for index, (ox, oy, radius) in enumerate(obstacles, start=1):
        ax.add_patch(
            Circle(
                (ox, oy),
                radius,
                facecolor="#E15759",
                edgecolor="#B52B30",
                linewidth=1.5,
                alpha=0.32,
                zorder=2,
            )
        )

        ax.add_patch(
            Circle(
                (ox, oy),
                radius + rover_radius,
                fill=False,
                edgecolor="#B52B30",
                linewidth=1.1,
                linestyle=":",
                alpha=0.9,
                zorder=2,
            )
        )

        ax.annotate(
            f"O{index}",
            xy=(ox, oy),
            xytext=(0, 10),
            textcoords="offset points",
            ha="center",
            fontsize=9,
            color="#8B2025",
            zorder=5,
        )

    # Home.
    ax.scatter(
        0,
        0,
        marker="*",
        s=300,
        color="#2878D0",
        edgecolor="white",
        linewidth=1,
        label="Home",
        zorder=7,
    )

    # Return-start position.
    if return_position is not None:
        ax.scatter(
            return_position[0],
            return_position[1],
            marker="s",
            s=110,
            color="#F28E2B",
            edgecolor="black",
            linewidth=0.8,
            label="Return start",
            zorder=7,
        )

    # Final position.
    ax.scatter(
        rover.x,
        rover.y,
        marker="o",
        s=125,
        color="#59A14F",
        edgecolor="white",
        linewidth=1.2,
        label="Final position",
        zorder=8,
    )

    # Mark actual avoidance starts and ends.
    start_label_used = False
    end_label_used = False

    for event_step, ex, ey, started in avoidance_events:
        if started:
            marker = "^"
            color = "#9467BD"
            label = (
                "Avoidance started"
                if not start_label_used
                else None
            )
            start_label_used = True
            offset = (8, 10)
        else:
            marker = "v"
            color = "#555555"
            label = (
                "Avoidance ended"
                if not end_label_used
                else None
            )
            end_label_used = True
            offset = (8, -14)

        ax.scatter(
            ex,
            ey,
            marker=marker,
            s=100,
            color=color,
            edgecolor="white",
            linewidth=0.8,
            label=label,
            zorder=9,
        )

        ax.annotate(
            f"{event_step}",
            (ex, ey),
            xytext=offset,
            textcoords="offset points",
            fontsize=8,
            color=color,
            zorder=9,
        )

    # Show any detected collision positions.
    if collision_events:
        ax.scatter(
            [event[2] for event in collision_events],
            [event[3] for event in collision_events],
            marker="X",
            s=110,
            color="black",
            edgecolor="white",
            linewidth=0.7,
            label="Collision detected",
            zorder=10,
        )

    ax.set_title(
        "Fruit Fly Brain Rover — Obstacle Avoidance",
        fontsize=16,
        fontweight="bold",
        pad=16,
    )

    ax.set_xlabel("X position (m)", fontsize=12)
    ax.set_ylabel("Y position (m)", fontsize=12)

    # Keep equal scaling so obstacle circles remain circular.
    all_x = trajectory_x + [x for x, _, _ in obstacles] + [0]
    all_y = trajectory_y + [y for _, y, _ in obstacles] + [0]

    padding = 0.8
    ax.set_xlim(min(all_x) - padding, max(all_x) + padding)
    ax.set_ylim(min(all_y) - padding, max(all_y) + padding)
    ax.set_aspect("equal", adjustable="box")

    ax.grid(
        True,
        linestyle="--",
        linewidth=0.7,
        alpha=0.25,
    )

    ax.set_facecolor("#FCFCFD")
    fig.patch.set_facecolor("white")

    ax.legend(
        loc="best",
        framealpha=0.96,
        edgecolor="#D0D0D0",
        fontsize=9,
    )

    # Intentionally no summary box on the chart.
    plt.tight_layout()

    output_path = (
        Path(__file__).resolve().parent.parent
        / "docs"
        / "avoidance_demo.png"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    fig.savefig(
        output_path,
        dpi=300,
        bbox_inches="tight",
        facecolor="white",
    )

    print(f"\nSaved visualization: {output_path}")
    plt.show()


if __name__ == "__main__":
    main()
