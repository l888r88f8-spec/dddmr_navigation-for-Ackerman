// Copyright 2026 DDDMobileRobot
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GRID_GLOBAL_PLANNER__METRICS_H_
#define GRID_GLOBAL_PLANNER__METRICS_H_

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>

#if defined(__linux__)
#include <sys/resource.h>
#include <time.h>
#endif

namespace grid_global_planner
{

// Schema version for every JSON record emitted on the telemetry topic. The
// benchmark collector and report tooling both read this value.
inline constexpr int kMetricsSchemaVersion = 1;

// Monotonic nanosecond timestamps. All in-process durations use this clock.
// ROS/sensor stamps remain separate and are never subtracted from it.
using SteadyNs = std::int64_t;

// Returns the current steady-clock instant in nanoseconds.
inline SteadyNs SteadyNowNs() noexcept
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch())
         .count();
}

// Converts an interval between two steady instants to milliseconds. A negative
// interval is reported as zero rather than a negative duration.
inline double SteadyDurationMs(SteadyNs start_ns, SteadyNs end_ns) noexcept
{
  if (end_ns <= start_ns) {
    return 0.0;
  }
  return static_cast<double>(end_ns - start_ns) / 1.0e6;
}

// Converts a steady-clock duration to milliseconds without losing sub-ms
// precision.
template<typename Duration>
inline double DurationToMs(Duration duration) noexcept
{
  return std::chrono::duration<double, std::milli>(duration).count();
}

// Coarse, opt-in resource usage sampled only at request/update boundaries.
// process_cpu_ms and peak_rss_mib cover the whole process; thread_cpu_ms covers
// the calling thread. They are coarse boundary snapshots, not per-phase
// attribution.
struct ProcessResourceSnapshot
{
  bool available{false};
  double process_cpu_ms{0.0};
  double thread_cpu_ms{0.0};
  double peak_rss_mib{0.0};
};

// Reads getrusage(RUSAGE_SELF) and CLOCK_THREAD_CPUTIME_ID on Linux. Other
// platforms return available=false so callers report the metric as unmeasured.
inline ProcessResourceSnapshot ReadProcessResourceSnapshot() noexcept
{
  ProcessResourceSnapshot snapshot;
#if defined(__linux__)
  struct rusage usage {};
  const bool process_available = ::getrusage(RUSAGE_SELF, &usage) == 0;
  if (process_available) {
    const double user_s = static_cast<double>(usage.ru_utime.tv_sec) +
      static_cast<double>(usage.ru_utime.tv_usec) / 1.0e6;
    const double system_s =
      static_cast<double>(usage.ru_stime.tv_sec) +
      static_cast<double>(usage.ru_stime.tv_usec) / 1.0e6;
    snapshot.process_cpu_ms = (user_s + system_s) * 1.0e3;
    // ru_maxrss is KiB on Linux. Convert to MiB explicitly.
    snapshot.peak_rss_mib =
      static_cast<double>(usage.ru_maxrss) / 1024.0;
  }
  struct timespec thread_time {};
  const bool thread_available =
    ::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_time) == 0;
  if (thread_available) {
    snapshot.thread_cpu_ms =
      static_cast<double>(thread_time.tv_sec) * 1.0e3 +
      static_cast<double>(thread_time.tv_nsec) / 1.0e6;
  }
  // A partial OS query must not make the other default-zero fields appear
  // measured. The aggregate record is available only when every field is valid.
  snapshot.available = process_available && thread_available;
#endif
  return snapshot;
}

// Per-build terrain telemetry. Fitting happens inside both segmentation and
// attribute extraction. segmentation_ms and attribute_ms are inclusive totals;
// the two *_fit_ms_inclusive values measure only the nested fit calls. The
// exclusive values are derived by subtraction and are never measured twice.
struct TerrainBuildMetrics
{
  bool measured{false};
  SteadyNs accepted_steady_ns{0};
  SteadyNs completed_steady_ns{0};
  double total_ms{0.0};
  double input_validation_ms{0.0};
  double content_hash_ms{0.0};
  double binning_ms{0.0};
  double segmentation_ms{0.0};
  double segmentation_fit_ms_inclusive{0.0};
  double attribute_ms{0.0};
  double attribute_fit_ms_inclusive{0.0};
  double connections_ms{0.0};
  // Derived, not separately measured: inclusive total minus nested fitting.
  double segmentation_fit_ms_exclusive{0.0};
  double attribute_fit_ms_exclusive{0.0};
  bool segmentation_includes_fitting{true};
  bool attribute_includes_fitting{true};
  std::size_t input_points{0};
  std::size_t valid_points{0};
  std::size_t cells{0};
  std::size_t surfaces{0};
  std::size_t connections{0};
  std::uint64_t revision{0};
  std::uint64_t content_hash{0};
  std::uint64_t observation_sequence{0};
  std::int64_t sensor_stamp_ns{0};
  std::int64_t received_steady_ns{0};
  std::string status;
  std::string detail;
};

// Per-evaluation four-wheel timing. All values are actual measured wall time
// for the named evaluator section; sample_count counts footprint samples that
// were physically queried.
struct WheelSupportTiming
{
  double total_ms{0.0};
  double footprint_ms{0.0};
  double branch_solve_ms{0.0};
  double pose_solve_ms{0.0};
  double body_check_ms{0.0};
  std::size_t sample_count{0};
  std::size_t branch_combinations{0};
  // Instrumented lookup counts. Body-envelope internals are not individually
  // counted, so callers must document these as lower bounds.
  std::size_t cell_queries{0};
  std::size_t surface_queries{0};
  std::size_t neighbor_queries{0};
  std::size_t obstacle_queries{0};
  std::size_t distance_queries{0};
};

// One worker event or one published snapshot. Instants are steady-clock values
// so queue wait and processing can be separated without mixing clock domains.
struct MapUpdateMetrics
{
  enum class Kind : std::uint8_t
  {
    kTerrainReplacement = 0,
    kSourceObservation,
    kSourceRemoval,
    kExpireSources,
    kPublish,
  };

  Kind kind{Kind::kSourceObservation};
  std::string source_id;
  std::uint64_t source_sequence{0};
  SteadyNs input_received_steady_ns{0};
  SteadyNs enqueued_steady_ns{0};
  SteadyNs started_steady_ns{0};
  SteadyNs completed_steady_ns{0};
  SteadyNs published_steady_ns{0};
  double queue_wait_ms{0.0};
  double total_ms{0.0};
  double contribution_ms{0.0};
  double cow_ms{0.0};
  double coarse_ms{0.0};
  double distance_ms{0.0};
  double snapshot_ms{0.0};
  double publish_ms{0.0};
  double reclaim_ms{0.0};
  std::uint64_t reclaimed_versions_total{0};
  std::size_t dirty_nodes{0};
  std::size_t dirty_cells{0};
  std::size_t updated_tiles{0};
  std::size_t copy_bytes_estimated{0};
  std::size_t live_versions{0};
  std::size_t retired_versions{0};
  bool version_counts_available{false};
  std::uint64_t version{0};
  bool publish_attempted{false};
  bool publish_succeeded{false};
  bool backpressured{false};
  TerrainBuildMetrics terrain;
  ProcessResourceSnapshot resources;
  std::string detail;
};

// Returns the stable spelling used in telemetry JSON.
inline const char * MapUpdateMetricKindToString(
  MapUpdateMetrics::Kind kind) noexcept
{
  switch (kind) {
    case MapUpdateMetrics::Kind::kTerrainReplacement:
      return "terrain_replacement";
    case MapUpdateMetrics::Kind::kSourceObservation:
      return "source_observation";
    case MapUpdateMetrics::Kind::kSourceRemoval:
      return "source_removal";
    case MapUpdateMetrics::Kind::kExpireSources:
      return "expire_sources";
    case MapUpdateMetrics::Kind::kPublish:
      return "publish";
  }
  return "unknown";
}

// Escapes a string for a JSON string literal. Control characters become
// \u00XX so a malformed diagnostic can never break the record.
inline std::string JsonEscape(std::string_view input)
{
  std::ostringstream output;
  output << std::hex << std::uppercase << std::setfill('0');
  for (const char character : input) {
    const unsigned char value = static_cast<unsigned char>(character);
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        if (value < 0x20U) {
          output << "\\u" << std::setw(4) << static_cast<unsigned int>(value);
        } else {
          output << character;
        }
        break;
    }
  }
  return output.str();
}

// Returns a quoted JSON string literal.
inline std::string JsonString(std::string_view input)
{
  return "\"" + JsonEscape(input) + "\"";
}

// Appends a finite JSON number, or null for non-finite values. The caller owns
// the surrounding field name.
inline std::string JsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream output;
  output << std::setprecision(17) << value;
  return output.str();
}

// Returns the current UTC wall time in ISO-8601 form. This is wall-clock
// provenance only and is never used to compute a duration.
inline std::string UtcNowIso8601()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm utc {};
#if defined(_WIN32)
  gmtime_s(&utc, &seconds);
#else
  gmtime_r(&seconds, &utc);
#endif
  const auto millis =
    std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) %
    1000;
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setw(3) << std::setfill('0') << millis.count() << 'Z';
  return output.str();
}

}  // namespace grid_global_planner

#endif  // GRID_GLOBAL_PLANNER__METRICS_H_
