#!/usr/bin/env python3
"""Replay real runtime DWA CSV traces recorded by OmniLocalPlanner."""

import argparse
import csv
import math
import warnings
from collections import defaultdict

warnings.filterwarnings(
    "ignore",
    message="Unable to import Axes3D.*",
    category=UserWarning,
)

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from matplotlib.patches import Circle, FancyArrowPatch


def load_world(path):
    world = {
        "origin": (0.0, 0.0),
        "width": 16.0,
        "height": 10.0,
        "goal": (0.0, 0.0),
        "path": [],
        "obstacles": [],
    }
    with open(path, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            kind = row["kind"]
            if kind == "bounds":
                world["origin"] = (float(row["x"]), float(row["y"]))
                world["width"] = float(row["width"])
                world["height"] = float(row["height"])
            elif kind == "goal":
                world["goal"] = (float(row["x"]), float(row["y"]))
            elif kind == "path":
                world["path"].append((float(row["x"]), float(row["y"])))
            elif kind == "obstacle":
                world["obstacles"].append((float(row["x"]), float(row["y"]), float(row["radius"])))
    return world


def load_trace(path):
    frames = defaultdict(
        lambda: {
            "robot": None,
            "prediction": [],
            "candidates": defaultdict(list),
            "score": {},
            "path": [],
            "costmap": [],
        }
    )
    with open(path, newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            step = int(row["step"])
            state = {
                "x": float(row["x"]),
                "y": float(row["y"]),
                "yaw": float(row["yaw"]),
                "vx": float(row["vx"]),
                "vy": float(row["vy"]),
                "wz": float(row["wz"]),
            }
            score = {
                "total": float(row["total_score"]),
                "path": float(row["path_distance"]),
                "target": float(row["target_distance"]),
                "goal": float(row["goal_distance"]),
                "obstacle": float(row["obstacle_score"]),
                "heading": float(row["heading_error"]),
                "velocity": float(row["velocity_error"]),
                "valid": row["valid"] == "1",
            }
            if row["kind"] == "robot":
                frames[step]["robot"] = state
                frames[step]["score"] = score
            elif row["kind"] == "prediction":
                frames[step]["prediction"].append((int(row["index"]), state))
            elif row["kind"] == "candidate":
                frames[step]["candidates"][int(row["candidate_id"])].append((int(row["index"]), state, score))
            elif row["kind"] == "path":
                frames[step]["path"].append((int(row["index"]), state))
            elif row["kind"] == "costmap":
                frames[step]["costmap"].append(state)

    ordered = []
    current_path = []
    for step in sorted(frames):
        frame = frames[step]
        frame["step"] = step
        frame["prediction"] = [state for _, state in sorted(frame["prediction"])]
        if frame["path"]:
            current_path = [state for _, state in sorted(frame["path"])]
        frame["path"] = current_path
        frame["candidates"] = [
            {
                "states": [state for _, state, _ in sorted(samples)],
                "score": sorted(samples)[0][2],
            }
            for _, samples in sorted(frame["candidates"].items())
        ]
        ordered.append(frame)
    return ordered


def calculate_trace_bounds(world, frames, use_world_bounds):
    if use_world_bounds:
        origin_x, origin_y = world["origin"]
        return origin_x, origin_x + world["width"], origin_y, origin_y + world["height"]

    xs = []
    ys = []
    for point in world["path"]:
        xs.append(point[0])
        ys.append(point[1])
    for x, y, radius in world["obstacles"]:
        xs.extend([x - radius, x + radius])
        ys.extend([y - radius, y + radius])
    if world["goal"] != (0.0, 0.0):
        xs.append(world["goal"][0])
        ys.append(world["goal"][1])

    for frame in frames:
        collections = [
            [frame["robot"]] if frame["robot"] else [],
            frame["prediction"],
            frame["path"],
            frame["costmap"],
        ]
        for candidate in frame["candidates"]:
            collections.append(candidate["states"])
        for states in collections:
            for state in states:
                xs.append(state["x"])
                ys.append(state["y"])

    if not xs or not ys:
        origin_x, origin_y = world["origin"]
        return origin_x, origin_x + world["width"], origin_y, origin_y + world["height"]

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max(max_x - min_x, 1.0)
    span_y = max(max_y - min_y, 1.0)
    margin = 0.08 * max(span_x, span_y)
    return min_x - margin, max_x + margin, min_y - margin, max_y + margin


class DwaPlot:
    def __init__(self, world, frames, robot_radius, use_world_bounds):
        self.world = world
        self.frames = frames
        self.robot_radius = robot_radius
        self.history_x = []
        self.history_y = []
        self.candidate_lines = []
        self.candidate_cmap = plt.get_cmap("viridis")
        self.candidate_norm = Normalize(vmin=0.0, vmax=1.0)

        self.fig, self.ax = plt.subplots(figsize=(10, 6), dpi=120)
        self.ax.set_aspect("equal", adjustable="box")
        min_x, max_x, min_y, max_y = calculate_trace_bounds(world, frames, use_world_bounds)
        self.ax.set_xlim(min_x, max_x)
        self.ax.set_ylim(min_y, max_y)
        self.ax.set_xlabel("x / m")
        self.ax.set_ylabel("y / m")
        self.ax.grid(True, alpha=0.25)
        self._draw_world()

        (self.history_line,) = self.ax.plot([], [], color="#0072B2", linewidth=2.0, label="robot path")
        (self.path_line,) = self.ax.plot(
            [], [], color="#666666", linewidth=1.5, linestyle=":", label="reference path"
        )
        (self.prediction_line,) = self.ax.plot(
            [], [], color="#D55E00", linewidth=2.0, linestyle="--", label="selected prediction"
        )
        self.ax.plot([], [], color=self.candidate_cmap(0.55), alpha=0.4, label="sampled candidates")
        self.costmap_scatter = self.ax.scatter([], [], s=10, c="#333333", alpha=0.28, marker="s", label="costmap")
        score_map = ScalarMappable(norm=self.candidate_norm, cmap=self.candidate_cmap)
        score_map.set_array([])
        self.fig.colorbar(score_map, ax=self.ax, fraction=0.035, pad=0.02, label="candidate score")
        self.robot_patch = Circle((0, 0), robot_radius, color="#009E73", alpha=0.85)
        self.ax.add_patch(self.robot_patch)
        self.heading_arrow = None
        self.score_text = self.ax.text(
            0.02,
            0.98,
            "",
            transform=self.ax.transAxes,
            va="top",
            ha="left",
            family="monospace",
            bbox={"facecolor": "white", "edgecolor": "#cccccc", "alpha": 0.85},
        )
        self.ax.legend(loc="lower right")

    def _draw_world(self):
        if self.world["path"]:
            (self.static_path_line,) = self.ax.plot(
                [point[0] for point in self.world["path"]],
                [point[1] for point in self.world["path"]],
                color="#666666",
                linewidth=1.5,
                linestyle=":",
                label="reference path",
            )
        self.ax.add_patch(Circle(self.world["goal"], 0.18, color="#CC79A7", label="goal", zorder=4))
        for index, (x, y, radius) in enumerate(self.world["obstacles"]):
            self.ax.add_patch(
                Circle((x, y), radius, color="#555555", alpha=0.65, label="obstacle" if index == 0 else None)
            )

    def update(self, frame_index):
        frame = self.frames[frame_index]
        robot = frame["robot"]
        prediction = frame["prediction"]
        score = frame["score"]
        path = frame["path"]
        costmap = frame["costmap"]

        for line in self.candidate_lines:
            line.remove()
        self.candidate_lines = []

        valid_candidates = [candidate for candidate in frame["candidates"] if candidate["score"].get("valid")]
        if valid_candidates:
            scores = [candidate["score"]["total"] for candidate in valid_candidates]
            score_min = min(scores)
            score_span = max(max(scores) - score_min, 1e-9)
            for candidate in valid_candidates:
                states = candidate["states"]
                normalized_score = (candidate["score"]["total"] - score_min) / score_span
                (line,) = self.ax.plot(
                    [state["x"] for state in states],
                    [state["y"] for state in states],
                    color=self.candidate_cmap(self.candidate_norm(1.0 - normalized_score)),
                    linewidth=1.0,
                    alpha=0.30,
                    zorder=2,
                )
                self.candidate_lines.append(line)

        self.history_x.append(robot["x"])
        self.history_y.append(robot["y"])
        self.history_line.set_data(self.history_x, self.history_y)
        self.path_line.set_data([state["x"] for state in path], [state["y"] for state in path])
        self.prediction_line.set_data([state["x"] for state in prediction], [state["y"] for state in prediction])
        if costmap:
            self.costmap_scatter.set_offsets([(state["x"], state["y"]) for state in costmap])
        else:
            self.costmap_scatter.set_offsets(np.empty((0, 2)))
        self.robot_patch.center = (robot["x"], robot["y"])

        if self.heading_arrow is not None:
            self.heading_arrow.remove()
        end_x = robot["x"] + 0.65 * math.cos(robot["yaw"])
        end_y = robot["y"] + 0.65 * math.sin(robot["yaw"])
        self.heading_arrow = FancyArrowPatch(
            (robot["x"], robot["y"]),
            (end_x, end_y),
            arrowstyle="-|>",
            mutation_scale=14,
            linewidth=2,
            color="#000000",
            zorder=5,
        )
        self.ax.add_patch(self.heading_arrow)

        self.score_text.set_text(
            f"step: {frame['step']}\n"
            f"pose: ({robot['x']:.2f}, {robot['y']:.2f}, yaw={robot['yaw']:.2f})\n"
            f"cmd: vx={robot['vx']:.2f} vy={robot['vy']:.2f} wz={robot['wz']:.2f}\n"
            f"score: {score['total']:.3f}\n"
            f"path: {score['path']:.3f}  target: {score['target']:.3f}\n"
            f"goal: {score['goal']:.3f}  obstacle: {score['obstacle']:.3f}\n"
            f"heading: {score['heading']:.3f}  speed: {score['velocity']:.3f}"
        )
        return self.history_line, self.prediction_line, self.robot_patch, self.score_text

    def draw_frame(self, frame_index):
        self.history_x = [frame["robot"]["x"] for frame in self.frames[: frame_index + 1] if frame["robot"]]
        self.history_y = [frame["robot"]["y"] for frame in self.frames[: frame_index + 1] if frame["robot"]]
        self.history_line.set_data(self.history_x, self.history_y)
        old_history_x = self.history_x
        old_history_y = self.history_y
        self.history_x = old_history_x[:-1]
        self.history_y = old_history_y[:-1]
        self.update(frame_index)
        self.fig.canvas.draw_idle()


def parse_args():
    parser = argparse.ArgumentParser(description="Visualize decoupled DWA CSV traces.")
    parser.add_argument("--world", default="dwa_world.csv")
    parser.add_argument("--trace", default="dwa_trace.csv")
    parser.add_argument("--robot-radius", type=float, default=0.22)
    parser.add_argument("--interval", type=int, default=60)
    parser.add_argument("--frame", type=int, default=None)
    parser.add_argument("--save", default=None)
    parser.add_argument(
        "--use-world-bounds",
        action="store_true",
        help="use bounds from dwa_world.csv instead of fitting the full recorded trace",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    world = load_world(args.world)
    frames = load_trace(args.trace)
    if not frames:
        raise RuntimeError("No frames found. Enable DWA trace and run navigation first.")

    plot = DwaPlot(world, frames, args.robot_radius, args.use_world_bounds)
    if args.frame is not None:
        frame_index = max(0, min(args.frame, len(frames) - 1))
        plot.draw_frame(frame_index)
        if args.save:
            plot.fig.savefig(args.save, bbox_inches="tight")
        else:
            plt.show()
        return

    anim = animation.FuncAnimation(
        plot.fig, plot.update, frames=len(frames), interval=args.interval, blit=False, repeat=False
    )
    if args.save:
        anim.save(args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
