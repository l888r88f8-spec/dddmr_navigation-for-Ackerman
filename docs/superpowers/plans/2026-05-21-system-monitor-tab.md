# System Monitor Tab Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a low-overhead System tab showing CPU, memory, and swap usage in `dddmr_web_control`.

**Architecture:** Extend the existing Python HTTP server with a `/api/system` endpoint backed by `/proc/stat` and `/proc/meminfo` helpers in `core.py`. Extend the existing static HTML/CSS/JS with a third tab that polls only while active.

**Tech Stack:** Python standard library, ROS 2 Python package layout, vanilla HTML/CSS/JavaScript, pytest.

---

### Task 1: Backend Metrics Helpers

**Files:**
- Modify: `src/dddmr_web_control/dddmr_web_control/core.py`
- Modify: `src/dddmr_web_control/test/test_core.py`

- [ ] Write failing pytest coverage for CPU tick parsing, CPU utilization delta calculation, and memory usage parsing.
- [ ] Run `pytest test/test_core.py -v` and verify the new tests fail because helpers do not exist.
- [ ] Implement `parse_proc_stat_cpu_ticks`, `calculate_cpu_usage`, `parse_proc_meminfo`, and `build_system_usage_payload`.
- [ ] Run `pytest test/test_core.py -v` and verify the tests pass.

### Task 2: HTTP Endpoint

**Files:**
- Modify: `src/dddmr_web_control/dddmr_web_control/web_control_node.py`
- Modify: `src/dddmr_web_control/test/test_core.py`

- [ ] Write failing static/behavior tests that require `/api/system` wiring and persistent CPU sample state.
- [ ] Run `pytest test/test_core.py -v` and verify the new tests fail.
- [ ] Add `WebControlNode.system_payload()` and route `GET /api/system` through the existing request handler.
- [ ] Run `pytest test/test_core.py -v` and verify the tests pass.

### Task 3: Frontend System Tab

**Files:**
- Modify: `src/dddmr_web_control/static/index.html`
- Modify: `src/dddmr_web_control/static/app.js`
- Modify: `src/dddmr_web_control/static/styles.css`
- Modify: `src/dddmr_web_control/test/test_core.py`

- [ ] Write failing static tests for the `System` tab, `/api/system` fetch, active-tab-only polling, and per-core bar container.
- [ ] Run `pytest test/test_core.py -v` and verify the new tests fail.
- [ ] Add the third tab, system viewport markup, JS polling/rendering, and CSS horizontal bars.
- [ ] Run `pytest test/test_core.py -v` and verify the tests pass.

### Task 4: Verification

**Files:**
- No new source files.

- [ ] Run `pytest test/test_core.py -v`.
- [ ] Run `python3 -m compileall dddmr_web_control`.
- [ ] Inspect `git diff --stat` and ensure only docs, tests, backend, and static assets changed.
