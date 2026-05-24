# System Monitor Tab Design

## Goal

Add a third tab to `dddmr_web_control` that displays low-overhead host resource usage: total CPU, per-core CPU, memory, and swap.

## Requirements

- Add a `System` tab next to `2D Console` and `3D Debug`.
- Show total CPU utilization and per-core CPU utilization.
- Render each CPU core as a horizontal bar.
- Show memory and swap used, total, and percent used.
- Minimize load on the measured host.

## Design

The backend exposes `GET /api/system`. It reads `/proc/stat` and `/proc/meminfo`, computes CPU utilization from the previous sample, and returns a compact JSON payload. The first CPU sample has no previous tick data, so CPU usage reports as `null` until the next sample.

The frontend polls `/api/system` only while the `System` tab is active. It polls once per second and immediately refreshes when the tab is opened. Rendering uses static DOM updates and CSS horizontal bars; no charting library or frontend build step is introduced.

## Testing

Unit tests cover CPU tick parsing, CPU delta calculations, memory parsing, and static frontend wiring for the third tab and system API usage.
