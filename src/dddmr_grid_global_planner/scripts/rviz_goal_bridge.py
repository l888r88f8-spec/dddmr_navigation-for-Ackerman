#!/usr/bin/env python3
# Copyright 2026 DDDMobileRobot
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""RViz start/goal click bridge for the grid global planner.

The bridge is visualization/planning-only and deliberately has no control path:

* ``/dddmr_grid_global_planner/rviz_start`` is a private
  ``PoseWithCovarianceStamped`` topic. Selecting a start pose only stores it
  locally and draws a marker. It is never republished as ``/initialpose`` and
  never touches localization.
* ``/dddmr_grid_global_planner/rviz_goal`` is a private ``PoseStamped`` topic.
  A goal click submits one ``dddmr_sys_core/action/GetPlan`` goal with
  ``activate_threading=true``. No cmd_vel, p2p, controller, or TF action is
  used.
* Only the latest goal matters. An older in-flight goal is canceled
  asynchronously and the pending goal is submitted only after the older goal
  reaches a terminal state. The planner rejects normal goals while busy, so
  submitting early would silently lose the click.
* At most one send future or accepted goal handle is retained. An ack that
  never arrives is marked timed out once and is never abandoned by dropping its
  future. A late accepted ack is canceled and tracked to a terminal state. An
  active goal that ignores cancellation is also tracked to a terminal state;
  new goals are not submitted before then. If the server never answers the
  ack, cancel, or result, only restarting the server (or this bridge) recovers.
* Result delivery is the only terminal release. A raised result future or a
  nonterminal/UNKNOWN status keeps the accepted handle tracked, requests
  cancellation once, and withholds all new goals until terminal evidence or a
  server/bridge restart. State is never released before that evidence exists.
* The bridge exclusively owns
  ``/dddmr_grid_global_planner/rviz_path`` for display. It publishes the action
  result path only for a successful current-generation goal with no newer
  pending goal. The planner's own ``~/path`` is left untouched.
* Every wait is a future callback plus a steady-clock watchdog timer.
  Callbacks never block the executor and pending work has a bounded lifetime.

Height policy (no silent guessing, no vehicle constants):

* ``start_height_mode``/``goal_height_mode`` are ``fixed`` or ``click``.
* The action pose position is the map-frame ``base_link`` reference position
  used by the planner. Its z is not a terrain surface height: the solver
  selects the supporting terrain layer and fits the pose from the request.
* ``fixed`` writes the corresponding ``*_fixed_z_m`` parameter. RViz 2D tools
  click at ``z=0`` in the fixed frame; the default 0.0 only makes that value
  explicit and is not automatically correct for a multilayer map. Set the
  start/goal fixed z from the map/layers and the vehicle reference.
* ``click`` keeps the z supplied by the RViz tool (normally 0.0).
* The bridge never infers z from terrain/TF and never replaces a missing start
  with TF. A TF-based empty start is only available by explicitly setting
  ``require_start=false``; that fallback is performed by the planner, not here.
* The project-confirmed ``vehicle.base_contact_height_m`` (0.22 m) is a planner
  vehicle parameter, not a bridge-side z offset; the bridge does not add it
  to the request.
"""

import copy
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from rcl_interfaces.msg import ParameterDescriptor
from builtin_interfaces.msg import Duration
from dddmr_sys_core.action import GetPlan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from visualization_msgs.msg import Marker, MarkerArray


# RViz tool publishers use reliable, volatile, keep-last QoS. The bridge must
# use a compatible subscription profile or the private click topics never
# match.
_TOOL_QOS = QoSProfile(
    depth=5,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _latched_qos(depth: int = 1) -> QoSProfile:
    """Returns a transient-local reliable profile for markers and path state.

    Args:
        depth: Keep-last history depth. Latched state only needs one sample.

    Returns:
        A QoS profile compatible with an RViz display configured as Transient
        Local / Reliable.
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


@dataclass
class PendingGoal:
    """One latest-wins goal waiting for the action server.

    Attributes:
        start: Resolved start pose, or None when the operator explicitly opted
            into the planner's empty-start TF fallback via ``require_start``.
        goal: Resolved goal pose. Only x/y/heading come from the RViz click;
            z is already resolved by the configured height policy.
        generation: Monotonic click sequence used to discard stale requests.
        created_at: ``time.monotonic()`` creation time for the submission
            watchdog.
    """

    start: Optional[PoseStamped]
    goal: PoseStamped
    generation: int
    created_at: float


class RvizGoalBridge(Node):
    """Bridges private RViz start/goal topics to ``/get_plan``.

    The node keeps one active action goal, one latest pending goal, and a
    marker/path visualization state. It intentionally does not use TF, publish
    localization updates, or send any vehicle command.
    """

    def __init__(self) -> None:
        super().__init__('dddmr_grid_rviz_bridge')

        # Parameters are read once at startup. Invalid values fail closed
        # before any subscription can accept a click.
        self._action_name = self._string_parameter(
            'action_name', '/get_plan')
        self._fixed_frame = self._string_parameter('fixed_frame', 'map')
        self._start_topic = self._string_parameter(
            'start_topic', '/dddmr_grid_global_planner/rviz_start')
        self._goal_topic = self._string_parameter(
            'goal_topic', '/dddmr_grid_global_planner/rviz_goal')
        self._marker_topic = self._string_parameter(
            'marker_topic', '/dddmr_grid_global_planner/rviz_markers')
        self._path_topic = self._string_parameter(
            'path_topic', '/dddmr_grid_global_planner/rviz_path')

        self._start_height_mode = self._height_mode_parameter(
            'start_height_mode')
        self._start_fixed_z = self._double_parameter(
            'start_fixed_z_m', 0.0)
        self._goal_height_mode = self._height_mode_parameter(
            'goal_height_mode')
        self._goal_fixed_z = self._double_parameter(
            'goal_fixed_z_m', 0.0)

        self._require_start = self._bool_parameter('require_start', True)
        self._submission_timeout_sec = self._positive_parameter(
            'submission_timeout_sec', 30.0)
        self._goal_timeout_sec = self._positive_parameter(
            'goal_timeout_sec', 60.0)
        self._cancel_grace_sec = self._positive_parameter(
            'cancel_grace_sec', 10.0)
        self._retry_delay_sec = self._positive_parameter(
            'retry_delay_sec', 0.5)
        self._status_log_period_sec = self._positive_parameter(
            'status_log_period_sec', 5.0)
        self._marker_z_offset_m = self._double_parameter(
            'marker_z_offset_m', 0.05)
        self._marker_scale_m = self._positive_parameter(
            'marker_scale_m', 0.8)
        self._warning_lifetime_sec = self._positive_parameter(
            'warning_lifetime_sec', 5.0)

        self._validate_parameters()

        # Local visualization state. A selected start is kept independently of
        # the active action goal so the operator can change it before the next
        # goal click.
        self._start_pose: Optional[PoseStamped] = None
        self._goal_marker_pose: Optional[PoseStamped] = None
        self._warning_text = ''
        self._warning_until = 0.0

        # Latest-wins action bookkeeping. Generations and attempt numbers make
        # late callbacks from canceled or timed-out requests harmless.
        self._generation = 0
        self._send_attempt = 0
        self._sent_generation = 0
        self._pending: Optional[PendingGoal] = None
        self._sent_request: Optional[PendingGoal] = None
        self._goal_future = None
        self._send_started_at: Optional[float] = None
        self._send_timed_out = False
        self._next_send_time = 0.0
        self._active_handle = None
        self._active_attempt = 0
        self._active_generation = 0
        self._active_started_at: Optional[float] = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._cancel_grace_logged = False
        self._last_wait_log = 0.0
        self._last_send_log = 0.0
        self._last_reject_log = 0.0
        self._shutting_down = False

        # Marker and path state are latched so RViz sees the latest selection
        # and the current-generation result. The bridge is the only publisher
        # of its dedicated rviz_path topic; the planner's ~/path is untouched.
        self._marker_publisher = self.create_publisher(
            MarkerArray, self._marker_topic, _latched_qos())
        self._path_publisher = self.create_publisher(
            Path, self._path_topic, _latched_qos())

        self._action_client = ActionClient(
            self, GetPlan, self._action_name)

        self._start_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self._start_topic,
            self._on_start_pose,
            _TOOL_QOS,
        )
        self._goal_subscription = self.create_subscription(
            PoseStamped,
            self._goal_topic,
            self._on_goal_pose,
            _TOOL_QOS,
        )
        # A steady clock keeps this watchdog alive when use_sim_time=true and
        # /clock is paused. Message stamps still use the node ROS clock.
        self._watchdog_timer = self.create_timer(
            0.1, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            'rviz bridge ready: action=%s start=%s goal=%s markers=%s '
            'path=%s fixed_frame=%s start_height=%s(%.3f) '
            'goal_height=%s(%.3f) require_start=%s' % (
                self._action_name, self._start_topic, self._goal_topic,
                self._marker_topic, self._path_topic, self._fixed_frame,
                self._start_height_mode, self._start_fixed_z,
                self._goal_height_mode, self._goal_fixed_z,
                self._require_start))
        if (self._start_height_mode == 'fixed' or
                self._goal_height_mode == 'fixed'):
            self.get_logger().info(
                'Fixed height policy: RViz 2D clicks are at z=0 in the fixed '
                'frame; this bridge writes the configured base_link reference '
                'z explicitly. Set the start/goal values from the map/layers '
                'and vehicle reference; 0.0 is not automatically correct for '
                'a multilayer map. No vehicle dimension is added here.')

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def _declare_read_only(self, name: str, default) -> None:
        """Declares a startup-only parameter that rejects dynamic changes."""
        descriptor = ParameterDescriptor(read_only=True)
        self.declare_parameter(name, default, descriptor=descriptor)

    def _string_parameter(self, name: str, default: str) -> str:
        """Declares and returns a non-empty string parameter."""
        self._declare_read_only(name, default)
        value = str(self.get_parameter(name).value)
        if not value:
            raise ValueError('%s must not be empty' % name)
        return value

    def _double_parameter(self, name: str, default: float) -> float:
        """Declares and returns a finite double parameter."""
        self._declare_read_only(name, float(default))
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value):
            raise ValueError('%s must be finite' % name)
        return value

    def _positive_parameter(self, name: str, default: float) -> float:
        """Declares and returns a finite, positive double parameter."""
        value = self._double_parameter(name, default)
        if value <= 0.0:
            raise ValueError('%s must be > 0' % name)
        return value

    def _bool_parameter(self, name: str, default: bool) -> bool:
        """Declares and returns a boolean parameter."""
        self._declare_read_only(name, bool(default))
        return bool(self.get_parameter(name).value)

    def _height_mode_parameter(self, name: str) -> str:
        """Declares and validates a height mode (``fixed`` or ``click``)."""
        self._declare_read_only(name, 'fixed')
        mode = str(self.get_parameter(name).value).strip().lower()
        if mode not in ('fixed', 'click'):
            raise ValueError(
                "%s must be 'fixed' or 'click'; got '%s'" % (name, mode))
        return mode

    def _validate_parameters(self) -> None:
        """Rejects unsafe combinations before the node accepts clicks."""
        if self._marker_z_offset_m < 0.0:
            raise ValueError('marker_z_offset_m must be >= 0')
        if self._submission_timeout_sec + self._retry_delay_sec <= 0.0:
            raise ValueError('submission/retry timeouts must be positive')

    # ------------------------------------------------------------------
    # RViz click callbacks
    # ------------------------------------------------------------------

    def _on_start_pose(self, message: PoseWithCovarianceStamped) -> None:
        """Stores a selected start locally and never triggers localization.

        A valid new start invalidates every older request: it increments the
        generation, drops a pending goal, clears the bridge-owned display path
        and old goal marker, and cancels an already accepted goal. It never
        submits a planning goal or publishes ``/initialpose``.

        Args:
            message: Pose published by RViz ``SetInitialPose`` on the private
                start topic. The pose is expected in ``fixed_frame``.
        """
        if self._shutting_down:
            return
        if not self._check_frame(message.header.frame_id, 'start'):
            return
        validated = self._validated_pose(
            message.pose.pose,
            message.header,
            'start',
            self._start_height_mode,
            self._start_fixed_z,
        )
        if validated is None:
            return
        start, source = validated

        if self._pending is not None:
            self.get_logger().info(
                'Dropping pending goal generation=%d because the start '
                'changed.' % self._pending.generation)
        self._pending = None
        # A send that is already in flight may still acknowledge. Its captured
        # generation no longer matches, so the late ack is canceled below.
        self._sent_request = None
        self._generation += 1
        self._start_pose = start
        self._goal_marker_pose = None
        self._warning_text = ''
        self._clear_path()
        self._publish_markers()
        if self._active_handle is not None:
            self._request_cancel(self._active_handle, self._active_attempt)
        self.get_logger().info(
            'start stored: x=%.3f y=%.3f z=%.3f (%s); generation=%d. No '
            'localization or action request was triggered. Click 2D Goal Pose '
            'to plan.' % (
                start.pose.position.x, start.pose.position.y,
                start.pose.position.z, source, self._generation))

    def _on_goal_pose(self, message: PoseStamped) -> None:
        """Stores a goal, clears stale path state, and queues latest-wins plan.

        Args:
            message: Pose published by RViz ``SetGoal`` on the private goal
                topic. Only x/y/heading are preserved; z is resolved by the
                configured policy.
        """
        if self._shutting_down:
            return
        if not self._check_frame(message.header.frame_id, 'goal'):
            return
        validated = self._validated_pose(
            message.pose,
            message.header,
            'goal',
            self._goal_height_mode,
            self._goal_fixed_z,
        )
        if validated is None:
            return
        goal, source = validated
        self._goal_marker_pose = goal

        if self._start_pose is None and self._require_start:
            # Do not cancel a valid in-flight plan or clear its path for a
            # click that cannot become a request. The old plan remains usable
            # until a start is selected and the goal is clicked again.
            self._warning_text = 'Select start first (2D Pose Estimate)'
            self._warning_until = time.monotonic() + self._warning_lifetime_sec
            self._publish_markers()
            self.get_logger().error(
                'Goal ignored: no start selected. Click 2D Pose Estimate '
                'first (the start is stored locally, not sent as '
                '/initialpose), then click 2D Goal Pose again. Set '
                'require_start:=false only when the planner TF fallback for '
                'an empty start is intended.')
            return

        self._generation += 1
        self._warning_text = ''
        self._clear_path()

        if self._start_pose is None:
            self.get_logger().warning(
                'No start selected and require_start=false: submitting an '
                'empty start. The planner will resolve map<-base_link through '
                'TF; this bridge does not perform that lookup.')
            start = None
        else:
            start = PoseStamped()
            start.header.frame_id = self._start_pose.header.frame_id
            start.header.stamp = self._start_pose.header.stamp
            start.pose = copy.deepcopy(self._start_pose.pose)

        pending = PendingGoal(
            start=start,
            goal=goal,
            generation=self._generation,
            created_at=time.monotonic(),
        )
        self._pending = pending
        self._publish_markers()
        self.get_logger().info(
            'goal queued: x=%.3f y=%.3f z=%.3f (%s) generation=%d' % (
                goal.pose.position.x, goal.pose.position.y,
                goal.pose.position.z, source, pending.generation))

        if self._active_handle is not None:
            # The planner rejects a second normal goal while busy. Cancel the
            # active goal now; its result callback will submit this pending
            # one.
            self._request_cancel(self._active_handle, self._active_attempt)

    def _check_frame(self, frame_id: str, role: str) -> bool:
        """Requires the RViz pose to already be in the configured fixed frame.

        Returning False instead of converting through TF keeps the bridge free
        of hidden transform dependencies. RViz publishes the fixed-frame pose
        for both 2D tools.

        Args:
            frame_id: Pose header frame from the RViz tool.
            role: Human-readable role used in the diagnostic.

        Returns:
            True when the pose frame is the configured fixed frame.
        """
        if frame_id == self._fixed_frame:
            return True
        self.get_logger().error(
            "%s frame '%s' does not match fixed_frame '%s'; no TF conversion "
            'is attempted. Set RViz Fixed Frame to %s and click again.' % (
                role, frame_id, self._fixed_frame, self._fixed_frame))
        return False

    def _validated_pose(
            self, pose, header, role: str, mode: str,
            fixed_z: float) -> Optional[tuple[PoseStamped, str]]:
        """Validates a click completely before any bridge state is mutated.

        Args:
            pose: ``geometry_msgs/Pose`` from the RViz tool.
            header: Source header for frame/stamp propagation.
            role: ``start`` or ``goal`` used in diagnostics.
            mode: ``fixed`` or ``click`` height policy.
            fixed_z: Configured reference z for ``fixed`` mode.

        Returns:
            A finite, normalized ``PoseStamped`` plus the height-source label,
            or None after logging a rejection. No bridge state is changed.
        """
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().error(
                'Rejecting %s click: position and quaternion must all be '
                'finite.' % role)
            return None
        try:
            z, source = self._resolve_height(
                pose.position.z, mode, fixed_z, role)
        except ValueError as error:
            self.get_logger().error(
                'Rejecting %s click: %s' % (role, error))
            return None

        result = PoseStamped()
        result.header.frame_id = self._fixed_frame
        result.header.stamp = header.stamp
        result.pose.position.x = float(pose.position.x)
        result.pose.position.y = float(pose.position.y)
        result.pose.position.z = z
        orientation = copy.deepcopy(pose.orientation)
        self._normalize_input_quaternion(orientation, role)
        result.pose.orientation = orientation
        return result, source

    def _resolve_height(
            self, clicked_z: float, mode: str, fixed_z: float,
            role: str) -> tuple[float, str]:
        """Resolves the base_link reference z from the explicit policy.

        Args:
            clicked_z: z supplied by the RViz tool (normally 0.0).
            mode: ``fixed`` or ``click``.
            fixed_z: Configured z for ``fixed`` mode.
            role: ``start`` or ``goal`` used in the returned description.

        Returns:
            A tuple of (z, human-readable source description).

        Raises:
            ValueError: If the selected mode or value is not finite.
        """
        if mode == 'click':
            z = float(clicked_z)
            source = '%s click z' % role
        else:
            z = float(fixed_z)
            source = '%s_fixed_z_m' % role
        if not math.isfinite(z):
            raise ValueError('%s z is not finite' % role)
        return z, source

    def _normalize_input_quaternion(self, orientation, role: str) -> None:
        """Normalizes a finite click quaternion in place.

        RViz normally sends a unit quaternion. If a tool ever sends zeros,
        writing w=1.0 keeps the request as an explicit zero-yaw map pose and
        prevents the planner from taking its zero-quaternion TF yaw fallback.

        Args:
            orientation: geometry_msgs Quaternion already checked finite.
            role: ``start`` or ``goal`` used in the warning.
        """
        norm = math.sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w)
        if norm < 1e-9:
            orientation.x = 0.0
            orientation.y = 0.0
            orientation.z = 0.0
            orientation.w = 1.0
            self.get_logger().warning(
                '%s click had an all-zero quaternion; using an explicit zero '
                'heading instead of a TF yaw fallback.' % role)
            return
        orientation.x /= norm
        orientation.y /= norm
        orientation.z /= norm
        orientation.w /= norm

    # ------------------------------------------------------------------
    # Latest-wins action state machine
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        """Drives dispatch and watchdogs without blocking the executor."""
        if self._shutting_down:
            return
        now = time.monotonic()

        # A pending goal must either be submitted or explicitly dropped. This
        # prevents an infinite queue when /get_plan never appears or another
        # client keeps the planner busy. A timed-out send future keeps blocking
        # dispatch until its late callback or terminal result is handled.
        if self._pending is not None:
            if now - self._pending.created_at > self._submission_timeout_sec:
                self.get_logger().error(
                    'Dropping goal generation=%d: not submitted within %.1f s '
                    '(server_ready=%s). The planner may be absent or busy; '
                    'click the goal again after it is ready.' % (
                        self._pending.generation,
                        self._submission_timeout_sec,
                        self._action_client.server_is_ready()))
                self._pending = None
            elif (self._goal_future is None and
                  self._active_handle is None and
                  not self._send_timed_out and now >= self._next_send_time):
                if self._action_client.server_is_ready():
                    self._send_pending()
                else:
                    self._log_server_wait(now)

        # Fail closed on a missing ack: mark it once, but keep the single
        # in-flight future. Dropping it would orphan an accepted handle and
        # allow duplicate goals. Pending goals still expire normally.
        if (self._goal_future is not None and
                self._send_started_at is not None and
                not self._send_timed_out and
                now - self._send_started_at > self._submission_timeout_sec):
            self._send_timed_out = True
            self.get_logger().error(
                'Goal generation=%d send did not acknowledge within %.1f s; '
                'keeping that one in-flight future and not submitting another '
                'request. A late accepted ack will be canceled and tracked to '
                'a terminal state. If the server never responds, recover or '
                'restart it.' % (
                    self._sent_generation, self._submission_timeout_sec))

        # Cancel an overlong active goal once. Never drop local tracking: a
        # terminal result is the only transition that releases the pipeline.
        if (self._active_handle is not None and
                self._active_started_at is not None and
                now - self._active_started_at > self._goal_timeout_sec):
            if not self._cancel_requested:
                self.get_logger().error(
                    'Active goal generation=%d exceeded '
                    'goal_timeout_sec=%.1f; requesting cancellation once and '
                    'keeping it tracked.' % (
                        self._active_generation, self._goal_timeout_sec))
                self._request_cancel(
                    self._active_handle, self._active_attempt)
            elif (not self._cancel_grace_logged and
                  now - self._active_started_at >
                  self._goal_timeout_sec + self._cancel_grace_sec):
                self._cancel_grace_logged = True
                self.get_logger().error(
                    'Active goal generation=%d is still not terminal after '
                    'cancel + %.1f s grace; continuing to track it and '
                    'withholding all new goals. Cancellation cannot be '
                    'guaranteed after a network failure; recover or restart '
                    'the action server (or restart this bridge).' % (
                        self._active_generation, self._cancel_grace_sec))

    def _log_server_wait(self, now: float) -> None:
        """Logs a throttled server-not-ready diagnostic."""
        if now - self._last_wait_log < self._status_log_period_sec:
            return
        self._last_wait_log = now
        if self._pending is not None:
            self.get_logger().warning(
                'Waiting for action server %s; goal generation=%d will be '
                'dropped after %.1f s.' % (
                    self._action_name, self._pending.generation,
                    self._submission_timeout_sec))
        else:
            self.get_logger().warning(
                'Action server %s is not ready.' % self._action_name)

    def _send_pending(self) -> None:
        """Submits the pending goal when the state machine allows it."""
        if (self._pending is None or self._goal_future is not None or
                self._active_handle is not None or self._send_timed_out):
            return
        if not self._action_client.server_is_ready():
            return
        now = time.monotonic()
        if now < self._next_send_time:
            return

        request = self._pending
        self._pending = None
        self._sent_request = request
        action_goal = self._build_action_goal(request)
        self._send_attempt += 1
        attempt = self._send_attempt
        self._sent_generation = request.generation
        self._send_started_at = now
        self._send_timed_out = False
        try:
            future = self._action_client.send_goal_async(action_goal)
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error(
                'send_goal_async raised: %s; the goal will be retried until '
                'the submission timeout.' % error)
            if self._pending is None:
                self._pending = request
            self._sent_request = None
            self._send_started_at = None
            self._next_send_time = now + self._retry_delay_sec
            return
        self._goal_future = future
        future.add_done_callback(
            lambda done, sent_attempt=attempt,
            sent_generation=request.generation: self._on_goal_response(
                done, sent_attempt, sent_generation))
        message = (
            'Sending /get_plan goal generation=%d activate_threading=true '
            '(latest-wins attempt=%d).' % (request.generation, attempt))
        if now - self._last_send_log >= self._status_log_period_sec:
            self._last_send_log = now
            self.get_logger().info(message)
        else:
            self.get_logger().debug(message)

    def _build_action_goal(self, request: PendingGoal) -> GetPlan.Goal:
        """Builds one GetPlan goal with fresh request timestamps.

        The stored start may have been selected minutes ago. The action request
        is stamped at submission time so the planner's sensor-age validation
        does not reject an operator-selected static pose. No geometry is
        changed except the explicitly configured z policy.

        Args:
            request: Pending latest-wins request.

        Returns:
            A GetPlan goal with ``activate_threading`` enabled.
        """
        goal = GetPlan.Goal()
        goal.activate_threading = True
        stamp = self.get_clock().now().to_msg()

        if request.start is not None:
            goal.start.header.frame_id = self._fixed_frame
            goal.start.header.stamp = stamp
            goal.start.pose = copy.deepcopy(request.start.pose)
        else:
            # Keep the action's empty-start semantic: empty frame, zero pose,
            # zero quaternion. The planner owns the explicit TF fallback.
            goal.start = PoseStamped()

        goal.goal.header.frame_id = self._fixed_frame
        goal.goal.header.stamp = stamp
        goal.goal.pose = copy.deepcopy(request.goal.pose)
        return goal

    def _on_goal_response(
            self, future, attempt: int, sent_generation: int) -> None:
        """Handles the asynchronous goal acceptance/rejection response."""
        if self._shutting_down or attempt != self._send_attempt:
            return
        self._goal_future = None
        self._send_started_at = None
        timed_out = self._send_timed_out
        self._send_timed_out = False
        try:
            goal_handle = future.result()
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error(
                'Goal generation=%d response failed: %s' % (
                    sent_generation, error))
            if timed_out:
                self._sent_request = None
            else:
                self._requeue_sent_request()
            self._next_send_time = time.monotonic() + self._retry_delay_sec
            return

        if not goal_handle.accepted:
            now = time.monotonic()
            message = (
                'Goal generation=%d rejected by %s (planner busy or goal '
                'invalid).' % (sent_generation, self._action_name))
            if now - self._last_reject_log >= self._status_log_period_sec:
                self._last_reject_log = now
                self.get_logger().warning(message)
            else:
                self.get_logger().debug(message)
            if timed_out:
                self._sent_request = None
                self.get_logger().warning(
                    'Timed-out goal generation=%d was rejected; it is not '
                    'retried. A newer pending goal may proceed.' %
                    sent_generation)
            else:
                self._requeue_sent_request()
            self._next_send_time = now + self._retry_delay_sec
            return

        self._sent_request = None
        self._active_handle = goal_handle
        self._active_attempt = attempt
        self._active_generation = sent_generation
        self._active_started_at = time.monotonic()
        self._cancel_requested = False
        self._cancel_grace_logged = False
        try:
            self._result_future = goal_handle.get_result_async()
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error(
                'get_result_async raised for generation=%d: %s; canceling and '
                'keeping the handle tracked. No new goal is submitted until a '
                'terminal result is available.' % (sent_generation, error))
            self._request_cancel(goal_handle, attempt)
            return
        self._result_future.add_done_callback(
            lambda done, active_attempt=attempt: self._on_result(
                done, active_attempt))
        self.get_logger().info(
            'Goal generation=%d accepted by %s.' % (
                self._active_generation, self._action_name))

        if timed_out or sent_generation != self._generation:
            self.get_logger().warning(
                'Canceling late/stale accepted goal generation=%d (current '
                'generation=%d, ack_timed_out=%s) and tracking it to a '
                'terminal state.' % (
                    sent_generation, self._generation, timed_out))
            self._request_cancel(goal_handle, attempt)
        elif self._pending is not None:
            # A newer click arrived while this goal was in flight.
            self._request_cancel(goal_handle, attempt)

    def _requeue_sent_request(self) -> None:
        """Restores the last submitted request only if it is still current."""
        request = self._sent_request
        if request is None:
            return
        self._sent_request = None
        if request.generation != self._generation:
            return
        if (self._pending is None or
                self._pending.generation < request.generation):
            self._pending = request

    def _request_cancel(self, goal_handle, attempt: int) -> None:
        """Requests cancellation once for the current accepted goal."""
        if (goal_handle is None or attempt != self._active_attempt or
                self._cancel_requested or self._cancel_future is not None):
            return
        self._cancel_requested = True
        try:
            self._cancel_future = goal_handle.cancel_goal_async()
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error(
                'cancel_goal_async raised: %s; waiting for the terminal '
                'result instead.' % error)
            self._cancel_future = None
            return
        self._cancel_future.add_done_callback(
            lambda done, active_attempt=attempt: self._on_cancel_response(
                done, active_attempt))
        self.get_logger().warning(
            'Canceling goal generation=%d; it stays tracked until a terminal '
            'result arrives. No new goal is submitted before then.' %
            self._active_generation)

    def _on_cancel_response(self, future, attempt: int) -> None:
        """Logs the asynchronous cancel acknowledgement."""
        if attempt != self._active_attempt:
            return
        self._cancel_future = None
        try:
            response = future.result()
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error('Cancel response failed: %s' % error)
            return
        if not response.goals_canceling:
            self.get_logger().warning(
                'Cancel response did not confirm a canceling goal; waiting '
                'for the terminal result before submitting a newer click.')

    def _on_result(self, future, attempt: int) -> None:
        """Handles terminal evidence and only then releases the pipeline."""
        if attempt != self._active_attempt:
            return
        generation = self._active_generation
        try:
            response = future.result()
        except Exception as error:  # pylint: disable=broad-except
            self._result_not_terminal(generation, attempt, str(error))
            return
        if response is None:
            self._result_not_terminal(
                generation, attempt, 'empty result response')
            return

        status = response.status
        if status not in (
                GoalStatus.STATUS_SUCCEEDED,
                GoalStatus.STATUS_CANCELED,
                GoalStatus.STATUS_ABORTED):
            self._result_not_terminal(
                generation, attempt, 'nonterminal status=%d' % status)
            return

        # Valid terminal evidence: only now is it safe to release the single
        # tracked handle and let the preserved latest pending click proceed.
        self._active_handle = None
        self._active_attempt = 0
        self._active_generation = 0
        self._active_started_at = None
        self._result_future = None
        self._cancel_future = None
        self._cancel_requested = False
        self._cancel_grace_logged = False
        self._next_send_time = time.monotonic() + self._retry_delay_sec

        path = None
        path_poses = 0
        if response.result is not None:
            path = response.result.path
            if path is not None:
                path_poses = len(path.poses)
        if status == GoalStatus.STATUS_SUCCEEDED:
            if (generation == self._generation and
                    self._pending is None and path is not None):
                self._path_publisher.publish(path)
                self.get_logger().info(
                    'Goal generation=%d succeeded: path_poses=%d; published '
                    'to %s.' % (generation, path_poses, self._path_topic))
            else:
                self.get_logger().info(
                    'Goal generation=%d succeeded but is not displayed '
                    '(current generation=%d, newer_pending=%s, '
                    'path_poses=%d).' % (
                        generation, self._generation,
                        self._pending is not None, path_poses))
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning(
                'Goal generation=%d canceled; the bridge-owned display path '
                'stays clear until a current-generation success.' % generation)
        else:
            self.get_logger().error(
                'Goal generation=%d finished with status=%d path_poses=%d; '
                'see the planner diagnostics topic for the failure reason.'
                % (generation, status, path_poses))

    def _result_not_terminal(
            self, generation: int, attempt: int, detail: str) -> None:
        """Keeps a goal tracked after result delivery fails to prove terminal."""
        self.get_logger().error(
            'Result for generation=%d did not prove a terminal state (%s); '
            'keeping the handle tracked, requesting cancellation once, and '
            'withholding all new goals until terminal evidence or recovery.' %
            (generation, detail))
        self._request_cancel(self._active_handle, attempt)

    # ------------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------------

    def _clear_path(self) -> None:
        """Publishes an empty latched path so a new request never shows old."""
        if self._path_publisher is None:
            return
        path = Path()
        path.header.frame_id = self._fixed_frame
        path.header.stamp = self.get_clock().now().to_msg()
        self._path_publisher.publish(path)

    def _publish_markers(self) -> None:
        """Publishes persistent start/goal arrows and an optional warning."""
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        if self._start_pose is None:
            markers.markers.append(self._delete_marker(0, stamp))
        else:
            markers.markers.append(
                self._arrow_marker(0, self._start_pose, (0.1, 0.9, 0.2)))
        if self._goal_marker_pose is None:
            markers.markers.append(self._delete_marker(1, stamp))
        else:
            markers.markers.append(
                self._arrow_marker(
                    1, self._goal_marker_pose, (1.0, 0.15, 0.1)))
        if (self._warning_text and
                time.monotonic() < self._warning_until and
                self._goal_marker_pose is not None):
            markers.markers.append(
                self._warning_marker(self._goal_marker_pose, stamp))
        else:
            markers.markers.append(self._delete_marker(2, stamp))
        self._marker_publisher.publish(markers)

    def _arrow_marker(
            self, marker_id: int, pose: PoseStamped,
            color: tuple[float, float, float]) -> Marker:
        """Builds a persistent arrow at a resolved start/goal pose."""
        marker = Marker()
        marker.header.frame_id = self._fixed_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rviz_start_goal'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z + self._marker_z_offset_m
        marker.pose.orientation = pose.pose.orientation
        self._normalize_orientation(marker.pose.orientation)
        marker.scale.x = self._marker_scale_m
        marker.scale.y = self._marker_scale_m * 0.12
        marker.scale.z = self._marker_scale_m * 0.22
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        return marker

    def _warning_marker(self, pose: PoseStamped, stamp) -> Marker:
        """Builds a short-lived text marker for the missing-start prompt."""
        marker = Marker()
        marker.header.frame_id = self._fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'rviz_start_goal'
        marker.id = 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = (
            pose.pose.position.z + self._marker_z_offset_m + 0.45)
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 1.0
        marker.text = self._warning_text
        marker.lifetime = Duration(
            sec=int(self._warning_lifetime_sec),
            nanosec=int(
                (self._warning_lifetime_sec % 1.0) * 1e9),
        )
        return marker

    def _delete_marker(self, marker_id: int, stamp) -> Marker:
        """Builds a marker-delete entry for an absent selection."""
        marker = Marker()
        marker.header.frame_id = self._fixed_frame
        marker.header.stamp = stamp
        marker.ns = 'rviz_start_goal'
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    @staticmethod
    def _normalize_orientation(orientation) -> None:
        """Normalizes a marker orientation for display only."""
        norm = math.sqrt(
            orientation.x * orientation.x +
            orientation.y * orientation.y +
            orientation.z * orientation.z +
            orientation.w * orientation.w)
        if norm < 1e-9:
            orientation.x = 0.0
            orientation.y = 0.0
            orientation.z = 0.0
            orientation.w = 1.0
            return
        orientation.x /= norm
        orientation.y /= norm
        orientation.z /= norm
        orientation.w /= norm

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def shutdown(self) -> None:
        """Cancels an active goal and removes this bridge's markers."""
        if self._shutting_down:
            return
        self._shutting_down = True
        try:
            self._watchdog_timer.cancel()
        except Exception:  # pylint: disable=broad-except
            pass
        if self._active_handle is not None:
            try:
                # The executor is about to stop. The request is sent
                # asynchronously; shutdown does not wait for a result.
                self._active_handle.cancel_goal_async()
                self.get_logger().info(
                    'Shutdown: asynchronous cancel requested for active goal.')
            except Exception as error:  # pylint: disable=broad-except
                self.get_logger().warning(
                    'Shutdown cancel request failed: %s' % error)
        try:
            markers = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            for marker_id in (0, 1, 2):
                markers.markers.append(self._delete_marker(marker_id, stamp))
            self._marker_publisher.publish(markers)
        except Exception:  # pylint: disable=broad-except
            # Context shutdown may already have invalidated the publisher.
            pass


def main(args=None) -> int:
    """Runs the bridge until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = RvizGoalBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:  # pylint: disable=broad-except
        if node is not None:
            node.get_logger().fatal('rviz bridge exception: %s' % error)
        else:
            print('rviz bridge startup exception: %s' % error, file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
