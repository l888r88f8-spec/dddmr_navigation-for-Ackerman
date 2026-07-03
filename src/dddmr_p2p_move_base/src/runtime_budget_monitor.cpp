#include "p2p_move_base/runtime_budget_monitor.h"

#include <cmath>
#include <sstream>

namespace p2p_move_base
{

RuntimeBudgetMonitor::RuntimeBudgetMonitor()
: RuntimeBudgetMonitor(Config())
{
}

RuntimeBudgetMonitor::RuntimeBudgetMonitor(const Config & config)
: config_(config), consecutive_overruns_(0)
{
}

void RuntimeBudgetMonitor::setConfig(const Config & config)
{
  config_ = config;
}

const RuntimeBudgetMonitor::Config & RuntimeBudgetMonitor::config() const
{
  return config_;
}

void RuntimeBudgetMonitor::reset()
{
  consecutive_overruns_ = 0;
  last_status_ = RuntimeBudgetStatus{};
}

bool RuntimeBudgetMonitor::exceedsBudget(
  const RuntimeBudgetSample & sample,
  std::string * reason) const
{
  std::ostringstream out;

  if(std::isfinite(sample.local_planning_time_ms) &&
    sample.local_planning_time_ms > config_.max_local_planning_time_ms)
  {
    out << "local_planning_time_ms=" << sample.local_planning_time_ms;
  } else if(std::isfinite(sample.obstacle_update_time_ms) &&
    sample.obstacle_update_time_ms > config_.max_obstacle_update_time_ms)
  {
    out << "obstacle_update_time_ms=" << sample.obstacle_update_time_ms;
  } else if(std::isfinite(sample.local_cycle_time_ms) &&
    sample.local_cycle_time_ms > config_.max_end_to_end_local_cycle_ms)
  {
    out << "local_cycle_time_ms=" << sample.local_cycle_time_ms;
  } else if(sample.candidate_trajectories > config_.max_candidate_trajectories) {
    out << "candidate_trajectories=" << sample.candidate_trajectories;
  } else if(sample.dynamic_obstacles > config_.max_dynamic_obstacles) {
    out << "dynamic_obstacles=" << sample.dynamic_obstacles;
  } else if(sample.snapshot_points > config_.max_points_per_snapshot) {
    out << "snapshot_points=" << sample.snapshot_points;
  } else if(sample.costmap_cells > config_.max_costmap_cells) {
    out << "costmap_cells=" << sample.costmap_cells;
  } else if(std::isfinite(sample.process_rss_mb) &&
    sample.process_rss_mb > config_.max_navigation_memory_mb)
  {
    out << "process_rss_mb=" << sample.process_rss_mb;
  } else {
    return false;
  }

  if(reason != nullptr){
    *reason = out.str();
  }
  return true;
}

RuntimeBudgetStatus RuntimeBudgetMonitor::update(const RuntimeBudgetSample & sample)
{
  std::string reason;
  const bool exceeded = exceedsBudget(sample, &reason);

  if(!exceeded){
    consecutive_overruns_ = 0;
    last_status_ = RuntimeBudgetStatus{};
    return last_status_;
  }

  ++consecutive_overruns_;

  last_status_.budget_exceeded = true;
  last_status_.consecutive_overruns = consecutive_overruns_;
  last_status_.reason = reason;

  if(consecutive_overruns_ >= config_.stop_after_consecutive_overruns){
    last_status_.level = BudgetLevel::SafetyStop;
  } else if(consecutive_overruns_ >= config_.warn_after_consecutive_overruns) {
    last_status_.level = BudgetLevel::Degrade;
  } else {
    last_status_.level = BudgetLevel::SoftOverrun;
  }

  return last_status_;
}

RuntimeBudgetStatus RuntimeBudgetMonitor::lastStatus() const
{
  return last_status_;
}

const char * toString(BudgetLevel level)
{
  switch(level){
    case BudgetLevel::Ok:
      return "ok";
    case BudgetLevel::SoftOverrun:
      return "soft_overrun";
    case BudgetLevel::Degrade:
      return "degrade";
    case BudgetLevel::SafetyStop:
      return "safety_stop";
  }
  return "unknown";
}

}  // namespace p2p_move_base
