
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from simulation.rover import Rover
from simulation.world import World


def main():
    # Seven obstacles in a staggered corridor.
    obstacles = [
        (2.0, 0.0, 0.55),
        (4.0, 0.8, 0.55),
        (6.0, -0.8, 0.55),
        (8.0, 0.0, 0.55),
        (10.0, 0.8, 0.55),
        (12.0, -0.8, 0.55),
    ]

    world = World(obstacles=obstacles)
    rover = Rover()

    return_step = 150
    total_steps = 750
    dt = 0.1

    trajectory_x = [rover.x]
    trajectory_y = [rover.y]

    return_position = None
    arrival_step = None

    print("=== FRUIT FLY BRAIN ROVER ===")
    print("Scenario: Staggered obstacle corridor")
    print(f"Obstacles: {len(obstacles)}")

    # --------------------------------------------------
    # Run the actual rover simulation.
    # --------------------------------------------------
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

        distance_home = math.hypot(rover.x, rover.y)

        # Log position, heading, commands, and avoidance state.
        if step % 25 == 0 or step == return_step:
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
                f"Speed={command.speed:.2f} | "
                f"Turn={command.turn_rate:.2f} | "
                f"Obstacle={obstacle_distance:.2f} m | "
                f"Avoiding={rover.avoiding_obstacle}"
            )

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
    print(f"Final position: ({rover.x:.3f}, {rover.y:.3f})")
    print(f"Final distance from home: {final_distance:.3f} m")
    print(f"Home reached: {arrival_step is not None}")

    # --------------------------------------------------
    # Build the visualization.
    # --------------------------------------------------
    fig, ax = plt.subplots(figsize=(14, 8))

    split = min(return_step + 1, len(trajectory_x))

    # Exploration trajectory.
    ax.plot(
        trajectory_x[:split],
        trajectory_y[:split],
        color="#2878D0",
        linewidth=2.8,
        label="Exploration",
        solid_capstyle="round",
        zorder=3,
    )

    # Return-home trajectory.
    ax.plot(
        trajectory_x[split - 1:],
        trajectory_y[split - 1:],
        color="#F28E2B",
        linewidth=2.8,
        label="Return-home",
        solid_capstyle="round",
        zorder=4,
    )

    # Obstacles and their physical radii.
    for ox, oy, radius in obstacles:
        ax.add_patch(
            Circle(
                (ox, oy),
                radius,
                facecolor="#E15759",
                edgecolor="#B52B30",
                linewidth=1.4,
                linestyle="--",
                alpha=0.27,
                zorder=2,
            )
        )

    # Home marker.
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

    # Return-start marker.
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

    # Actual final-position marker.
    ax.scatter(
        rover.x,
        rover.y,
        marker="o",
        s=115,
        color="#59A14F",
        edgecolor="white",
        linewidth=1,
        label="Final position",
        zorder=8,
    )

    # Add direction arrows at several points along each phase.
    def add_direction_arrows(xs, ys, color, max_arrows=4):
        if len(xs) < 2:
            return

        moving_segments = []

        for i in range(len(xs) - 1):
            distance = math.hypot(
                xs[i + 1] - xs[i],
                ys[i + 1] - ys[i],
            )

            if distance > 1e-6:
                moving_segments.append(i)

        if not moving_segments:
            return

        arrow_count = min(max_arrows, len(moving_segments))

        for k in range(arrow_count):
            selected = round(
                k * (len(moving_segments) - 1)
                / max(1, arrow_count - 1)
            )
            i = moving_segments[selected]

            ax.annotate(
                "",
                xy=(xs[i + 1], ys[i + 1]),
                xytext=(xs[i], ys[i]),
                arrowprops={
                    "arrowstyle": "-|>",
                    "color": color,
                    "lw": 1.8,
                    "mutation_scale": 15,
                },
                zorder=6,
            )

    add_direction_arrows(
        trajectory_x[:split],
        trajectory_y[:split],
        "#2878D0",
    )

    add_direction_arrows(
        trajectory_x[split - 1:],
        trajectory_y[split - 1:],
        "#F28E2B",
    )

    # Titles and axes.
    ax.set_title(
        "Fruit Fly Brain Rover — Staggered Obstacle Course",
        fontsize=17,
        fontweight="bold",
        pad=15,
    )

    ax.set_xlabel("X position (m)", fontsize=12)
    ax.set_ylabel("Y position (m)", fontsize=12)

    # Fit the plot to the trajectory and obstacle field.
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
        alpha=0.22,
    )

    ax.set_facecolor("#FCFCFD")
    fig.patch.set_facecolor("white")

    ax.legend(
        loc="upper right",
        frameon=True,
        framealpha=0.95,
        edgecolor="#D0D0D0",
        fontsize=10,
    )

    plt.tight_layout()

    # Save the visualization.
    output_path = (
        Path(__file__).resolve().parent.parent
        / "docs"
        / "alternate_scenario_visualization.png"
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
