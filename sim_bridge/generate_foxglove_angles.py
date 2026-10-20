#!/usr/bin/env python3
"""Generate the official Foxglove programmatic layout for auto-aim angles.

Optional generation dependency only: pip install foxglove-sdk==0.29.2
Reference: https://foxglove-sdk-api-docs.pages.dev/python/notebook/panels/Plot
"""

import json
from dataclasses import replace
from pathlib import Path

from foxglove.layouts import Layout, PlotConfig, PlotPanel, PlotSeries, SplitContainer, SplitItem

TOPIC = "/auto_aim/metrics"
COLORS = {"target": "#4E79A7", "gimbal": "#F28E2B", "plan": "#59A14F"}


def series(field, label, color):
    return PlotSeries(
        value=f"{TOPIC}.{field}",
        label=label,
        color=color,
        enabled=True,
        show_line=True,
        line_size=2,
        timestamp_method="customField",
        timestamp_path=f"{TOPIC}.timestamp",
    )


def plot(title, paths):
    return PlotPanel(
        title=title,
        config=PlotConfig(
            paths=paths,
            x_axis_val="timestamp",
            x_axis_display_method="relative",
            x_axis_label="Time (s)",
            y_axis_label="Angle (deg)",
            time_window_mode="sliding",
            following_view_width=20,
            playback_bar_position="right",
            is_synced=True,
            show_legend=True,
            legend_display="top",
            show_plot_values_in_legend=True,
            show_x_axis_labels=True,
            show_y_axis_labels=True,
            show_y_axis_grid_lines=True,
        ),
    )


def build_panels():
    yaw = plot("Yaw: target / gimbal / plan", [
        series(f"{name}.yaw_unwrapped_deg", f"{name}.yaw", COLORS[name])
        for name in ("target", "gimbal", "plan")
    ])
    pitch = plot("Pitch: target / gimbal / plan", [
        series(f"{name}.pitch_deg", f"{name}.pitch", COLORS[name])
        for name in ("target", "gimbal", "plan")
    ])
    error = plot("Aim error: yaw / pitch", [
        series("error.yaw_deg", "error.yaw", "#E15759"),
        series("error.pitch_deg", "error.pitch", "#B07AA1"),
    ])
    return {"yaw": yaw, "pitch": pitch, "error": error}


def build_layout():
    panels = build_panels()
    return Layout(content=SplitContainer(direction="column", items=[
        SplitItem(content=SplitContainer(direction="row", items=[
            SplitItem(content=panels["yaw"]), SplitItem(content=panels["pitch"]),
        ])),
        SplitItem(content=panels["error"]),
    ]))


if __name__ == "__main__":
    layout = build_layout()
    # The SDK owns the versioned JSON schema; do not hand-construct an opaque
    # configById layout from these panel settings.
    serialized = json.loads(layout.to_json())
    destination = Path(__file__).with_name("foxglove_angles.json")
    destination.write_text(json.dumps(serialized, indent=2) + "\n", encoding="utf-8")
    print(destination)
    settings_directory = destination.with_name("foxglove_plots")
    settings_directory.mkdir(exist_ok=True)
    for name, panel in build_panels().items():
        # These are SDK-generated Plot settings, suitable for each Plot panel's
        # Import/Export Settings dialog. Receive time also works in older apps;
        # all compared curves arrive in the same /auto_aim/metrics message.
        compatible_paths = [replace(path, timestamp_method="receiveTime", timestamp_path=None)
                            for path in panel.config.paths]
        config = replace(panel.config, paths=compatible_paths)
        settings = settings_directory / f"{name}.json"
        settings.write_text(json.dumps(json.loads(config.to_json()), indent=2) + "\n",
                            encoding="utf-8")
        print(settings)
