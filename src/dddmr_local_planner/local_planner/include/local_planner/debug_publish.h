/*
 * Diagnostic publish gate.
 *
 * Diagnostic topics are outside the control path: nothing downstream of the
 * planner reads them, so the only reason to serialize and send one is an
 * actual subscriber. Centralising the decision keeps every debug publisher on
 * the same rule and makes it testable without a ROS graph.
 */

#ifndef LOCAL_PLANNER_DEBUG_PUBLISH_H_
#define LOCAL_PLANNER_DEBUG_PUBLISH_H_

#include <cstddef>

namespace lp
{

inline bool shouldPublishDebug(bool enabled, std::size_t subscriber_count)
{
  return enabled && subscriber_count > 0u;
}

}  // namespace lp

#endif  // LOCAL_PLANNER_DEBUG_PUBLISH_H_
