#ifndef P2P_MOVE_BASE__PUBLISH_FEEDBACK_IF_ACTIVE_H_
#define P2P_MOVE_BASE__PUBLISH_FEEDBACK_IF_ACTIVE_H_

namespace p2p_move_base
{

// Publish action feedback only while the goal can still consume it.
//
// The stopping predicate is evaluated both before publishing and again if the
// publish call throws. That second check distinguishes a feedback transport
// failure while the goal is still live (propagate to the execution error path)
// from a preemption/cancellation/shutdown that raced the publish (normal
// termination handoff, no feedback sent).
template<typename GoalHandle, typename IsStopping, typename Publish>
bool PublishFeedbackIfActive(
  const GoalHandle & goal,
  IsStopping && is_stopping,
  Publish && publish)
{
  const auto may_publish =
    [&goal, &is_stopping]() {
      return goal && goal->is_active() && !goal->is_canceling() && !is_stopping();
    };

  if(!may_publish()){
    return false;
  }

  try{
    publish();
  }
  catch(...){
    if(may_publish()){
      throw;
    }
    return false;
  }
  return true;
}

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__PUBLISH_FEEDBACK_IF_ACTIVE_H_
