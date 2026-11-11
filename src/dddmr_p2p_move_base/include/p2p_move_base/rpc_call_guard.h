#ifndef P2P_MOVE_BASE_RPC_CALL_GUARD_H
#define P2P_MOVE_BASE_RPC_CALL_GUARD_H

#include <utility>

namespace p2p_move_base
{
/**
 * Runs one ROS RPC only while the owning node context is valid.
 *
 * @param context_is_valid Callable returning true while the node context remains valid.
 * @param rpc Callable performing the RPC; it may throw.
 * @return true when the RPC was attempted, false when it was skipped because the
 *         context was already invalid.
 *
 * The context is checked again after a failure. A failure raised while the
 * context is still valid is rethrown to the caller. A failure caused by the
 * context becoming invalid while the call is in flight (shutdown race) is
 * suppressed because the client library is already tearing down.
 */
template<typename ContextIsValid, typename Rpc>
bool invokeRpcIfContextValid(ContextIsValid && context_is_valid, Rpc && rpc)
{
  if(!context_is_valid()){
    return false;
  }

  try{
    std::forward<Rpc>(rpc)();
  }
  catch(...){
    if(context_is_valid()){
      throw;
    }
    // Context shut down while the RPC was in flight; there is nothing left to
    // cancel against and suppressing avoids turning shutdown into a failure.
  }
  return true;
}

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE_RPC_CALL_GUARD_H
