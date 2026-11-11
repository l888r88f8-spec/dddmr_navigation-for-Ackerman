/*
* BSD 3-Clause License
*
* Copyright (c) 2024, DDDMobileRobot
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef P2P_MOVE_BASE__PAUSE_AWARE_WATCHDOG_H_
#define P2P_MOVE_BASE__PAUSE_AWARE_WATCHDOG_H_

namespace p2p_move_base
{

struct WatchdogUpdate
{
  bool paused = false;
  bool resumed = false;
  bool expired = false;
};

// Deadline tracking for control loops that must not count time while sensor or
// TF data is unavailable.  The caller reports a data fault on every affected
// cycle; the deadline is frozen for the whole fault and restarts on the first
// healthy cycle.  By default a non-positive patience disables expiration,
// matching the existing "oscillation_patience <= 0" convention; pass false for
// non_positive_disables when zero must remain an immediate deadline (blocked
// wait did not support a disabled timeout before).  Not thread-safe; expected
// to be owned by the single navigation execution thread.
class PauseAwareWatchdog
{
public:
  explicit PauseAwareWatchdog(
    double patience_sec = 0.0,
    bool non_positive_disables = true)
  : patience_sec_(patience_sec),
    non_positive_disables_(non_positive_disables)
  {
  }

  void setPatience(double patience_sec) noexcept
  {
    patience_sec_ = patience_sec;
  }

  // Restart the deadline, for example after real motion progress or when
  // entering a new phase.  Also establishes the baseline on first use.
  void reset(double now_sec) noexcept
  {
    last_reset_sec_ = now_sec;
    started_ = true;
    paused_ = false;
  }

  // Call once per control cycle after the controller state is known.  While
  // data_fault is true the deadline cannot expire.  The first non-fault cycle
  // reports resumed and restarts the deadline without expiring.
  WatchdogUpdate update(bool data_fault, double now_sec) noexcept
  {
    WatchdogUpdate result;

    if (!started_) {
      last_reset_sec_ = now_sec;
      started_ = true;
    }

    if (data_fault) {
      paused_ = true;
      last_reset_sec_ = now_sec;
      result.paused = true;
      return result;
    }

    if (paused_) {
      paused_ = false;
      last_reset_sec_ = now_sec;
      result.resumed = true;
      return result;
    }

    if (patience_sec_ <= 0.0 && non_positive_disables_) {
      return result;
    }

    const double elapsed_sec = now_sec - last_reset_sec_;
    if (elapsed_sec >= patience_sec_) {
      result.expired = true;
    }
    return result;
  }

private:
  double patience_sec_;
  bool non_positive_disables_;
  double last_reset_sec_ = 0.0;
  bool paused_ = false;
  bool started_ = false;
};

}  // namespace p2p_move_base

#endif  // P2P_MOVE_BASE__PAUSE_AWARE_WATCHDOG_H_
