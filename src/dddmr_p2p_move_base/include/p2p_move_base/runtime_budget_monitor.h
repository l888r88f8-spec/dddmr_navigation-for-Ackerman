#ifndef P2P_MOVE_BASE_RUNTIME_BUDGET_MONITOR_H_
#define P2P_MOVE_BASE_RUNTIME_BUDGET_MONITOR_H_

#include <cstddef>
#include <string>

namespace p2p_move_base
{

enum class BudgetLevel
{
  Ok,
  SoftOverrun,
  Degrade,
  SafetyStop
};

struct RuntimeBudgetSample
{
  double obstacle_update_time_ms = 0.0;
  double local_planning_time_ms = 0.0;
  double local_cycle_time_ms = 0.0;
  std::size_t snapshot_points = 0;
  std::size_t costmap_cells = 0;
  std::size_t candidate_trajectories = 0;
  std::size_t dynamic_obstacles = 0;
  double process_rss_mb = 0.0;
};

struct RuntimeBudgetStatus
{
  BudgetLevel level = BudgetLevel::Ok;
  bool budget_exceeded = false;
  std::size_t consecutive_overruns = 0;
  std::string reason;
};

class RuntimeBudgetMonitor
{
public:
  struct Config
  {
    double max_local_planning_time_ms = 50.0;
    double max_obstacle_update_time_ms = 50.0;
    double max_end_to_end_local_cycle_ms = 80.0;
    std::size_t max_candidate_trajectories = 80;
    std::size_t max_dynamic_obstacles = 48;
    std::size_t max_points_per_snapshot = 120000;
    std::size_t max_costmap_cells = 120000;
    double max_navigation_memory_mb = 1200.0;
    std::size_t warn_after_consecutive_overruns = 2;
    std::size_t stop_after_consecutive_overruns = 10;
  };

  RuntimeBudgetMonitor();
  explicit RuntimeBudgetMonitor(const Config & config);

  void setConfig(const Config & config);
  const Config & config() const;
  void reset();

  RuntimeBudgetStatus update(const RuntimeBudgetSample & sample);
  RuntimeBudgetStatus lastStatus() const;

private:
  Config config_;
  RuntimeBudgetStatus last_status_;
  std::size_t consecutive_overruns_;

  bool exceedsBudget(const RuntimeBudgetSample & sample, std::string * reason) const;
};

const char * toString(BudgetLevel level);

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_RUNTIME_BUDGET_MONITOR_H_
