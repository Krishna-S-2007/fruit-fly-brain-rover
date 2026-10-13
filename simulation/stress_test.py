
import json
from pathlib import Path

from simulation.evaluate import generate_extra_scenarios, run_scenario


SEEDS = [7, 21, 42, 73, 101, 123, 256, 512, 999, 2026]
SCENARIOS_PER_SEED = 20


def main():
    all_results = []

    print("\n=== MULTI-SEED ROBUSTNESS STRESS TEST ===")
    print(f"Random seeds: {SEEDS}")
    print(f"Scenarios per seed: {SCENARIOS_PER_SEED}")
    print(f"Total scenarios: {len(SEEDS) * SCENARIOS_PER_SEED}\n")

    for seed in SEEDS:
        scenarios = generate_extra_scenarios(
            count=SCENARIOS_PER_SEED,
            seed=seed,
        )

        seed_results = []

        for scenario_name, obstacles in scenarios.items():
            result = run_scenario(
                f"Seed {seed} - {scenario_name}",
                obstacles,
            )
            seed_results.append(result)

        arrivals = sum(
            result["arrival"] for result in seed_results
        )

        collision_scenarios = sum(
            result["collision_steps"] > 0
            for result in seed_results
        )

        passed = sum(
            result["arrival"]
            and result["collision_steps"] == 0
            for result in seed_results
        )

        total_collisions = sum(
            result["collision_steps"]
            for result in seed_results
        )

        all_results.extend(seed_results)

        print(
            f"Seed {seed:4d} | "
            f"Arrivals={arrivals}/{len(seed_results)} | "
            f"Passed={passed}/{len(seed_results)} | "
            f"Collision scenarios={collision_scenarios} | "
            f"Collision steps={total_collisions}"
        )

    total = len(all_results)

    arrivals = sum(
        result["arrival"] for result in all_results
    )

    passed = sum(
        result["arrival"]
        and result["collision_steps"] == 0
        for result in all_results
    )

    collision_scenarios = sum(
        result["collision_steps"] > 0
        for result in all_results
    )

    collision_steps = sum(
        result["collision_steps"]
        for result in all_results
    )

    summary = {
        "seeds": SEEDS,
        "scenarios_per_seed": SCENARIOS_PER_SEED,
        "total_scenarios": total,
        "home_arrivals": arrivals,
        "home_arrival_rate_percent": round(
            100 * arrivals / total, 2
        ) if total else 0.0,
        "fully_passed_scenarios": passed,
        "pass_rate_percent": round(
            100 * passed / total, 2
        ) if total else 0.0,
        "scenarios_with_collision": collision_scenarios,
        "total_collision_steps": collision_steps,
        "results": all_results,
    }

    print("\n=== OVERALL SUMMARY ===")
    print(f"Scenarios: {total}")
    print(f"Home arrivals: {arrivals}/{total}")
    print(f"Fully passed: {passed}/{total}")
    print(
        f"Pass rate: "
        f"{summary['pass_rate_percent']:.2f}%"
    )
    print(f"Scenarios with collisions: {collision_scenarios}")
    print(f"Total collision steps: {collision_steps}")

    failed = [
        result
        for result in all_results
        if not result["arrival"]
        or result["collision_steps"] > 0
    ]

    if failed:
        print("\n=== FAILED SCENARIOS ===")

        for result in failed:
            print(
                f"{result['name']} | "
                f"Home={result['arrival']} | "
                f"Distance={result['final_distance_m']} m | "
                f"Collisions={result['collision_steps']} | "
                f"Obstacles={result['obstacles']}"
            )
    else:
        print("\nAll stress-test scenarios passed.")

    output_path = Path("docs") / "stress_test_results.json"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )

    print(f"\nResults saved to: {output_path}")


if __name__ == "__main__":
    main()
