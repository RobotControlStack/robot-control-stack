"""Plot parameterized box slip ablation CSV files.

Each input CSV is expected to use the filename written by ``run_ablate.py``:

    box_z_records_<slide>_<spin>_<roll>_<density>_seed_<seed>.csv

Older files without the ``_seed_<seed>`` suffix are also supported; for those,
the seed is read from the CSV contents.

One fixed-axis PNG is written per CSV so images can be compared directly.
"""

from __future__ import annotations

import argparse
import csv
import re
from dataclasses import dataclass
from pathlib import Path

_CSV_RE = re.compile(
    r"^box_z_records_"
    r"(?P<friction_slide>[^_]+)_"
    r"(?P<friction_spin>[^_]+)_"
    r"(?P<friction_roll>[^_]+)_"
    r"(?P<density>[^_]+)"
    r"(?:_seed_(?P<seed>-?\d+))?\.csv$"
)


@dataclass(frozen=True)
class AblationParams:
    friction_slide: float
    friction_spin: float
    friction_roll: float
    density: float
    seed: int


def _parse_filename_float(value: str) -> float:
    return float(value.replace("m", "-").replace("p", "."))


def _parse_csv_params(csv_path: Path) -> AblationParams:
    match = _CSV_RE.match(csv_path.name)
    if match is None:
        msg = f"Could not parse ablation parameters from {csv_path.name!r}."
        raise ValueError(msg)

    seed = match.group("seed")
    return AblationParams(
        friction_slide=_parse_filename_float(match.group("friction_slide")),
        friction_spin=_parse_filename_float(match.group("friction_spin")),
        friction_roll=_parse_filename_float(match.group("friction_roll")),
        density=_parse_filename_float(match.group("density")),
        seed=int(seed) if seed is not None else _read_seed_from_csv(csv_path),
    )


def _read_seed_from_csv(csv_path: Path) -> int:
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            seed = row.get("seed")
            if seed not in (None, ""):
                return int(seed)

    msg = f"Could not find a seed column value in {csv_path}."
    raise ValueError(msg)


def _read_ablation_csv(csv_path: Path) -> tuple[list[int], list[float], list[float]]:
    steps: list[int] = []
    z_drops: list[float] = []
    gripper_widths: list[float] = []

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row["phase"] != "step":
                continue
            steps.append(int(row["step"]))
            z_drops.append(float(row["z_drop_from_reset"]))
            gripper_widths.append(float(row["commanded_gripper_width"]))

    if not steps:
        msg = f"No step rows found in {csv_path}."
        raise ValueError(msg)
    return steps, z_drops, gripper_widths


def _first_slip(
    *,
    steps: list[int],
    z_drops: list[float],
    gripper_widths: list[float],
    slip_threshold: float,
) -> tuple[int, float] | None:
    for step, z_drop, gripper_width in zip(steps, z_drops, gripper_widths, strict=True):
        if z_drop >= slip_threshold:
            return step, gripper_width
    return None


def _plot_csv(
    *,
    csv_path: Path,
    png_path: Path,
    params: AblationParams,
    x_limit: int,
    y_min: float,
    y_max: float,
    slip_threshold: float,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except ModuleNotFoundError as exc:
        msg = "plot_ablation.py requires matplotlib. Install it in the Python environment used for plotting."
        raise SystemExit(msg) from exc

    steps, z_drops, gripper_widths = _read_ablation_csv(csv_path)
    first_slip = _first_slip(
        steps=steps,
        z_drops=z_drops,
        gripper_widths=gripper_widths,
        slip_threshold=slip_threshold,
    )

    fig, ax = plt.subplots(figsize=(12, 6), dpi=160)
    ax.plot(steps, z_drops, color="#1f77b4", linewidth=1.8, label="z drop from reset")
    ax.axhline(
        slip_threshold,
        color="#d62728",
        linestyle="--",
        linewidth=1.2,
        label=f"slip threshold ({slip_threshold:g} m)",
    )

    ax.set_xlim(0, x_limit)
    ax.set_ylim(y_min, y_max)
    ax.set_xlabel("control step")
    ax.set_ylabel("z drop from reset (m)")
    ax.grid(True, alpha=0.25)

    gripper_ax = ax.twinx()
    gripper_ax.plot(
        steps,
        gripper_widths,
        color="#2ca02c",
        linewidth=1.0,
        alpha=0.65,
        label="commanded gripper width",
    )
    gripper_ax.set_ylim(0.0, 1.0)
    gripper_ax.set_ylabel("commanded gripper width")

    slip_text = (
        f"first slip: step {first_slip[0]}, width {first_slip[1]:.3f}"
        if first_slip is not None
        else "first slip: none"
    )
    title = (
        "Box slip ablation\n"
        f"friction=({params.friction_slide:g}, {params.friction_spin:g}, {params.friction_roll:g}), "
        f"density={params.density:g}, seed={params.seed}; {slip_text}"
    )
    ax.set_title(title)

    lines, labels = ax.get_legend_handles_labels()
    gripper_lines, gripper_labels = gripper_ax.get_legend_handles_labels()
    ax.legend(lines + gripper_lines, labels + gripper_labels, loc="upper right")

    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(png_path)
    plt.close(fig)


def plot_ablation(args: argparse.Namespace) -> None:
    input_dir = args.input_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    csv_paths = sorted(path for path in input_dir.glob("box_z_records_*.csv") if _CSV_RE.match(path.name))
    if not csv_paths:
        msg = f"No parameterized ablation CSV files found in {input_dir}."
        raise FileNotFoundError(msg)

    for csv_path in csv_paths:
        params = _parse_csv_params(csv_path)
        png_path = output_dir / f"{csv_path.stem}.png"
        _plot_csv(
            csv_path=csv_path,
            png_path=png_path,
            params=params,
            x_limit=args.x_limit,
            y_min=args.y_min,
            y_max=args.y_max,
            slip_threshold=args.slip_threshold,
        )
        print(f"Wrote {png_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot fixed-axis box slip ablation CSV files.")
    parser.add_argument("--input-dir", type=Path, default=Path("ablation_runs"))
    parser.add_argument("--output-dir", type=Path, default=Path("ablation_plots"))
    parser.add_argument("--x-limit", type=int, default=3500)
    parser.add_argument("--y-min", type=float, default=-0.005)
    parser.add_argument("--y-max", type=float, default=0.05)
    parser.add_argument("--slip-threshold", type=float, default=0.01)

    args = parser.parse_args()
    plot_ablation(args)


if __name__ == "__main__":
    main()
