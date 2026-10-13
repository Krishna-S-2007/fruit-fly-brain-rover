
from simulation.rover import Rover
from simulation.world import World


def test_rover_recovers_when_avoidance_times_out():
    world = World(obstacles=[(0.5, 0.0, 0.3)])
    rover = Rover()
    rover.max_avoidance_steps = 3

    initial_position = (rover.x, rover.y)

    for _ in range(30):
        rover.step(world, dt=0.1)

    final_position = (rover.x, rover.y)

    assert final_position != initial_position, (
        "Rover remained stationary despite avoidance recovery."
    )
    assert rover.no_progress_steps < 30, (
        "Rover made no positional progress for every step."
    )
