//! Machine + control state machine (`MachineState`/`ControlState` and their outcomes).

use super::*;

/// The machine run-state reported as the first field of a `<...>` status report. Stage 1 only ever
/// reports [`Idle`](MachineState::Idle), but the full grblHAL set is enumerated here so the formatter and
/// the shared [`MachineSnapshot`] are Stage-2-ready (alarm/hold/homing) without a breaking change.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MachineState {
  /// No motion queued or executing.
  Idle,
  /// Executing a queued motion block.
  Run,
  /// Feed hold: `Hold:0` complete/ready-to-resume, `Hold:1` in progress. The `bool` is the substate.
  Hold(bool),
  /// Executing a jog.
  Jog,
  /// Halted in an alarm; the `u8` is the grblHAL alarm code (added to the full `0x87` report).
  Alarm(u8),
  /// Safety door open.
  Door,
  /// `$C` check mode: parse/validate without moving.
  Check,
  /// Running a homing cycle.
  Home,
  /// `$SLP` sleep: spindle/coolant off, drivers parked, held until a soft reset wakes the machine.
  Sleep,
  /// grblHAL `STATE_TOOLCHANGE`: an `M6` manual tool change is held, awaiting a cycle-start (`~`) resume. A bare
  /// state token with no substate — distinct from a feed-hold's `Hold:0` (M0/M1 still report `Hold:0`).
  Tool,
}

impl MachineState {
  /// The grblHAL status-report state token, written as the first field of a `<...>` report. Substates
  /// (`Hold:0`/`Hold:1`, `Alarm:<code>`) are appended by the status formatter, not encoded here.
  pub(crate) fn token(self) -> &'static str {
    match self {
      MachineState::Idle => "Idle",
      MachineState::Run => "Run",
      MachineState::Hold(_) => "Hold",
      MachineState::Jog => "Jog",
      MachineState::Alarm(_) => "Alarm",
      MachineState::Door => "Door",
      MachineState::Check => "Check",
      MachineState::Home => "Home",
      MachineState::Sleep => "Sleep",
      MachineState::Tool => "Tool",
    }
  }
}

/// The authoritative, latched control mode of the machine — the single source of truth the `firmware` bin
/// shares across its comms tasks and the status reporter (DOC-08 Stage 2). It is deliberately SMALLER than
/// [`MachineState`]: the Run-vs-Idle distinction in [`MachineState`] is *derived* from live execution facts
/// (in-flight block count) at report time via [`ControlState::machine_state`], not latched here, so two
/// tasks never race to write "Run". The `$`/real-time handlers mutate THIS; the status formatter renders the
/// composed [`MachineState`]. Kept a pure, `Copy` state machine so every transition is host-tested.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ControlState {
  /// Normal operation: the machine reports `Idle` when no block is in flight and `Run` while executing. This
  /// is the only state whose reported [`MachineState`] depends on live execution.
  Normal,
  /// Feed-hold active (`!`/`0x82`). The `bool` is the grbl substate: `false` = `Hold:0` (stopped, ready to
  /// resume), `true` = `Hold:1` (still decelerating). Phase A pauses at the block boundary, so it latches
  /// `Hold:0` directly; the `Hold:1` substate is reserved for the smooth ramp-down refinement.
  Hold(bool),
  /// A `$J=` jog is in flight (DOC-08 Phase D). Latched when a jog is accepted and held until the jog blocks
  /// drain (or a jog-cancel `0x85` flushes them), at which point the machine returns to `Normal`. Like
  /// `Normal`, the reported [`MachineState`] depends on live execution: `Jog` while jog blocks run, `Idle` once
  /// they drain. A jog NEVER changes modal/coordinate state, so leaving `Jog` needs no reset side-effects.
  Jog,
  /// Halted in an alarm; carries the [`AlarmCode`]. Entered on boot-lock (`$22`), soft-reset-during-cycle,
  /// and later phases' limit/probe/e-stop events. Cleared by `$X` (non-locked codes) or a soft reset.
  Alarm(AlarmCode),
  /// `$C` check mode: GCode is parsed and validated (and `ok`'d) but NOT planned or executed. Toggled off by
  /// a second `$C`, which grbl follows with a soft reset.
  Check,
  /// `$SLP` sleep: spindle/coolant off, drivers parked; held until a soft reset wakes the machine (DOC-07/
  /// DOC-03 own the actual peripheral shutdown — the state machine only latches the mode).
  Sleep,
  /// An `M6` manual tool change is held, awaiting a cycle-start (`~`) resume (grblHAL `STATE_TOOLCHANGE`). Reports
  /// the dedicated `Tool` wire state rather than `Hold:0` (M0/M1 still latch `Hold(false)`). Like a hold it is
  /// cycle-start-resumable (back to `Normal`) and motion-allowed (so the resumed program continues), but it is a
  /// DISTINCT wire state so a sender shows a tool-change prompt rather than a generic pause.
  Tool,
}

impl ControlState {
  /// The boot state: locked in [`AlarmCode::HomingRequired`] when `$22` homing is enabled (a host must `$H`
  /// or `$X` before streaming), otherwise [`Normal`](ControlState::Normal). Mirrors grbl's power-on behavior.
  pub fn boot(homing_enabled: bool) -> Self {
    if homing_enabled {
      ControlState::Alarm(AlarmCode::HomingRequired)
    } else {
      ControlState::Normal
    }
  }

  /// Compose the reported [`MachineState`] from this latched mode plus whether a block is currently in flight
  /// (queued or mid-burst on the executor). Only [`Normal`](ControlState::Normal) consults `running`: it
  /// reports `Run` while a block executes and `Idle` otherwise. Every other mode maps to its fixed state, so
  /// the Run/Idle derivation never has to race the latched modes. This is the one place `?` turns control
  /// state into a wire state.
  pub fn machine_state(self, running: bool) -> MachineState {
    match self {
      ControlState::Normal => {
        if running {
          MachineState::Run
        } else {
          MachineState::Idle
        }
      }
      // A jog reports `Jog` while its blocks run and `Idle` once they drain — like `Normal`, the Run/Idle
      // (here Jog/Idle) split is derived from live execution, never latched, so the reporter and the executor
      // never race to write it. The consumer drops `Jog` back to `Normal` once the queue empties.
      ControlState::Jog => {
        if running {
          MachineState::Jog
        } else {
          MachineState::Idle
        }
      }
      ControlState::Hold(in_progress) => MachineState::Hold(in_progress),
      ControlState::Alarm(code) => MachineState::Alarm(code.code()),
      ControlState::Check => MachineState::Check,
      ControlState::Sleep => MachineState::Sleep,
      // An M6 manual tool change reports the dedicated `Tool` state (latched, independent of live execution).
      ControlState::Tool => MachineState::Tool,
    }
  }

  /// Enter the `M6` manual-tool-change hold (`Tool` state). A constructor (not a transition from another mode) so
  /// the consumer's M6 pause sets it explicitly; `~` (cycle-start) resumes it back to `Normal` via
  /// [`cycle_start`](ControlState::cycle_start), and motion stays allowed so the resumed program continues.
  pub fn tool_change() -> Self {
    ControlState::Tool
  }

  /// Whether GCode motion is currently allowed. False in any alarm, in sleep, and in check mode (check parses
  /// and `ok`s but never plans). The consumer gates planning on this so a held/alarmed/asleep machine never
  /// enqueues a move. (Hold does not block *planning* — blocks may queue while held; the executor pauses at
  /// the boundary — so `Hold` is motion-allowed here.)
  pub fn motion_allowed(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Hold(_) | ControlState::Tool)
  }

  /// Apply a feed-hold (`!`/`0x82`): from `Normal` (or an existing hold) latch `Hold:0`. A hold requested in
  /// any other mode (alarm/check/sleep) is a no-op — grbl ignores `!` when not running a program. Returns the
  /// resulting state so the caller can publish it.
  pub fn feed_hold(self) -> Self {
    match self {
      ControlState::Normal | ControlState::Hold(_) => ControlState::Hold(false),
      other => other,
    }
  }

  /// Apply a cycle-start (`~`/`0x81`): release a feed-hold back to `Normal` (the Run/Idle distinction is then
  /// re-derived from live execution). In any non-hold mode it is a no-op, matching grbl (`~` does not clear an
  /// alarm or wake from sleep — only a soft reset / `$X` does).
  pub fn cycle_start(self) -> Self {
    match self {
      // Both a feed-hold and an M6 tool-change hold resume to `Normal` on `~` (Run/Idle re-derived from execution).
      ControlState::Hold(_) | ControlState::Tool => ControlState::Normal,
      other => other,
    }
  }

  /// Whether a cycle-start (`~`/`0x81`) should resume motion from THIS control state — i.e. whether the machine
  /// is in a feed-hold that `~` releases. `~` resumes ONLY a hold: it is inert in Idle/Run (`Normal`), Jog,
  /// Alarm, Check, and Sleep. Sleep in particular must NOT wake on `~` — only a soft reset wakes a sleeping
  /// machine (a `$SLP` parks the executor via the same hold latch, but a `~` must leave it parked). Pulled out
  /// as a pure predicate so the bin's real-time `~` dispatch can gate the executor-hold release on it without
  /// duplicating the state logic, and so the gate is host-tested here rather than in the untestable wiring.
  pub fn resumes_on_cycle_start(self) -> bool {
    // A feed-hold (M0/M1) AND an M6 tool-change hold both resume on `~`; every other mode is inert (an alarm/sleep
    // only clears via `$X`/soft reset).
    matches!(self, ControlState::Hold(_) | ControlState::Tool)
  }

  /// Whether a hard-limit trip from the core-1 executor should raise a fresh `ALARM:1` from THIS control state.
  /// True from the states where the executor's EDGE-armed detector can produce a genuine NEW assertion worth
  /// surfacing: `Normal` (running a program), `Hold` (a held program could resume into a switch), `Jog`, and
  /// `Check`. `Check` never enqueues motion, but the trip is edge-armed (`hard_limit_alarm_armed`), so it fires
  /// only on a switch NEWLY pressed DURING the dry-run — a real safety event grbl surfaces regardless of mode, not
  /// a stale parked-switch read. FALSE from any `Alarm(_)` and from `Sleep`: in those states the machine is
  /// already halted/parked, so a trip is a STALE read of a parked switch. Re-raising `ALARM:1` over an existing alarm changes
  /// nothing useful and can only CLOBBER a more-specific state — most damagingly downgrading the boot-lock
  /// `ALARM:11` (homing required) into the locked `ALARM:1`, losing the "homing required" semantic the host must
  /// satisfy. Pulled out as a pure predicate so the bin's hard-limit consumer arm can gate the `ALARM:1` raise
  /// without duplicating the alarm/sleep logic, and so the gate is host-tested here rather than in the untestable
  /// cross-core wiring. The PRIMARY fix for the post-aborted-homing `error:9` re-lock is the executor's EDGE-armed
  /// hard-limit alarm (`hard_limit_alarm_armed`): a switch left engaged after a `$H` abort is a HELD level, not a
  /// fresh edge, so it never signals a stale trip in the first place. This predicate is the single remaining
  /// DEFENSIVE layer (the soft-reset signal drains were removed once the arming subsumed them): it ensures any
  /// stray `HARD_LIMIT_TRIPPED` arriving while ALREADY alarmed/asleep cannot downgrade a more-specific lock (most
  /// damagingly `ALARM:11`) into the locked `ALARM:1`. A legitimately NEW over-travel still alarms because the
  /// machine is in `Normal`/`Hold`/`Jog`/`Check` while moving, and the executor re-signals on its fresh edge.
  pub fn hard_limit_alarm_applies(self) -> bool {
    matches!(
      self,
      ControlState::Normal | ControlState::Hold(_) | ControlState::Jog | ControlState::Check
    )
  }

  /// Apply a soft reset (`0x18`). Per grbl: a reset that aborts an IN-PROGRESS cycle raises
  /// [`AlarmCode::AbortDuringCycle`] (position is suspect after a mid-move halt); a reset from any other
  /// state returns to the boot state — locked in homing-required when `$22` is set, else `Normal`. A reset
  /// also clears an existing (non-homing) alarm, check, or sleep back to the boot baseline. `was_in_cycle`
  /// is the executor's "a block was mid-flight when the reset landed" fact.
  pub fn soft_reset(self, was_in_cycle: bool, homing_enabled: bool) -> Self {
    if was_in_cycle {
      // Aborting a move loses positional certainty: grbl forces an abort alarm regardless of `$22` so the
      // host must re-establish state (re-home or `$X`) before the next move.
      ControlState::Alarm(AlarmCode::AbortDuringCycle)
    } else {
      ControlState::boot(homing_enabled)
    }
  }

  /// Apply `$X` (kill alarm lock). Valid only from a NON-locked alarm (the locked critical codes 1/2/10 are
  /// cleared by a soft reset, not `$X`): it returns `Normal` and the caller emits `[MSG:Caution: Unlocked]`.
  /// From a locked alarm it stays put and the caller rejects the command. From a non-alarm state `$X` is a
  /// no-op `ok` (returns `self` unchanged); the [`Unlock`](UnlockOutcome) result distinguishes the cases.
  pub fn unlock(self) -> (Self, UnlockOutcome) {
    match self {
      ControlState::Alarm(code) if code.is_locked() => (self, UnlockOutcome::Locked),
      ControlState::Alarm(_) => (ControlState::Normal, UnlockOutcome::Unlocked),
      other => (other, UnlockOutcome::NotAlarmed),
    }
  }

  /// Toggle `$C` check mode. From `Normal` it enters [`Check`](ControlState::Check) (the caller emits
  /// `[MSG:Enabled]`); from `Check` it leaves check mode, which grbl realizes as a soft reset — so this
  /// returns the post-reset boot state and signals the caller (via [`CheckToggle::Disabled`]) to run the
  /// reset side-effects (banner, parser/planner rebuild). From any other mode it is rejected (returns `self`
  /// with [`CheckToggle::Rejected`]) — grbl only enters check mode from Idle/Normal.
  pub fn toggle_check(self, homing_enabled: bool) -> (Self, CheckToggle) {
    match self {
      ControlState::Normal => (ControlState::Check, CheckToggle::Enabled),
      ControlState::Check => (ControlState::boot(homing_enabled), CheckToggle::Disabled),
      other => (other, CheckToggle::Rejected),
    }
  }

  /// Apply `$SLP` (sleep). Allowed from `Normal` only (grbl rejects sleep while alarmed or running a hold):
  /// latches [`Sleep`](ControlState::Sleep), and the caller parks the spindle/drivers (DOC-07/DOC-03) and
  /// holds the pipeline until a soft reset wakes the machine. From any other mode it is rejected.
  pub fn enter_sleep(self) -> (Self, bool) {
    match self {
      ControlState::Normal => (ControlState::Sleep, true),
      other => (other, false),
    }
  }

  /// Whether a `$J=` jog may be accepted now (DOC-08 Phase D). grbl accepts a jog ONLY from Idle or an existing
  /// jog (so a stream of `$J=` lines chains smoothly), and rejects it while running a program, in a feed-hold,
  /// or in any alarm/check/sleep state. `Normal` here means Idle-or-Run; the consumer additionally requires the
  /// program queue to be empty (no program block in flight) before accepting, so this gates the LATCHED mode and
  /// the consumer gates live execution — together they realize grbl's "jog only from Idle/Jog".
  pub fn jog_allowed(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Jog)
  }

  /// Whether a settings-mutating `$` command (`$n=val`, `$Nx=`, `$RST=`, the `$PBX` import) may be accepted
  /// now. grbl accepts these ONLY while IDLE or ALARMED and rejects them with `error:8` otherwise — settings
  /// must never change under a running job (the planner/executor would mix old- and new-scale kinematics), and
  /// Alarm is allowed so a bad value (say a soft-limit travel) can be corrected without unlocking first. Like
  /// [`jog_allowed`](ControlState::jog_allowed), this gates the LATCHED mode only: `Normal` means Idle-or-Run,
  /// so the consumer additionally requires live motion to be quiescent — together they realize grbl's
  /// "settings only when idle". Hold/Jog/Check/Sleep/Tool are rejected outright, as in grbl.
  pub fn settings_write_allowed(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Alarm(_))
  }

  /// Latch [`Jog`](ControlState::Jog) on accepting a `$J=` jog. From `Normal` or an existing `Jog` it enters
  /// (or stays in) `Jog`; from any other mode it is a no-op (returns `self`) — the consumer only calls this
  /// after [`jog_allowed`](ControlState::jog_allowed) clears, so the no-op arm is purely defensive.
  pub fn begin_jog(self) -> Self {
    match self {
      ControlState::Normal | ControlState::Jog => ControlState::Jog,
      other => other,
    }
  }

  /// Return to `Normal` on a jog-cancel (`0x85`) or once the jog blocks drain (DOC-08 Phase D). A jog NEVER
  /// changed modal/coordinate state, so leaving `Jog` needs no reset side-effects — it simply drops the latch
  /// back to `Normal` (whose reported state re-derives Idle/Run from live execution). From any non-jog mode it
  /// is a no-op (`0x85` is ignored when not jogging), returning `self`.
  pub fn cancel_jog(self) -> Self {
    match self {
      ControlState::Jog => ControlState::Normal,
      other => other,
    }
  }

  /// Apply a graceful program stop (`0x86`, Galdr extension): from `Normal` (running or idle) or either `Hold`
  /// substate, return to [`Normal`](ControlState::Normal) — a motion-capable Idle once motion drains. NEVER an
  /// alarm: this is the controlled "stop the job" the operator wants, distinct from the `0x18` abort that raises
  /// `ALARM:3` on a mid-cycle reset and loses positional certainty. From any other mode (alarm, check, sleep, or
  /// an in-flight jog — which has its own `0x85` cancel) it is a benign no-op (returns `self`). A program stop
  /// never changes the coordinate model or loses position, so leaving the running/held state needs no alarm and
  /// the position is retained; the bin clears the modal/program-run state separately (mirroring `M30`).
  pub fn program_stop(self) -> Self {
    match self {
      ControlState::Normal | ControlState::Hold(_) => ControlState::Normal,
      other => other,
    }
  }

  /// Whether a graceful program stop (`0x86`) needs the executor parked at a block boundary and the planner queue
  /// flushed — i.e. whether the machine is in a state that could have a program running or held. True for `Normal`
  /// (which may be executing a program) and `Hold`; false for the inert states (alarm, check, sleep) and for `Jog`
  /// (a jog is not a program — it has its own `0x85` cancel). Pulled out as a pure predicate so the bin gates its
  /// boundary-quiesce + flush work on it without duplicating the state logic, and so the gate is host-tested here.
  pub fn program_stop_quiesces(self) -> bool {
    matches!(self, ControlState::Normal | ControlState::Hold(_))
  }

  /// Whether a `$H` homing cycle may START from this state (DOC-06). grbl runs `$H` from Idle/Normal AND from
  /// the homing-required boot alarm (`$H` is THE way to clear `ALARM:11`), but refuses it from the locked
  /// critical alarms (hard/soft limit, e-stop — those need a soft reset first), from a feed-hold, and from
  /// check/sleep. Pulled out as a predicate so the bin gates `$H` on it without duplicating the state logic.
  pub fn homing_allowed(self) -> bool {
    match self {
      ControlState::Normal => true,
      // The boot lock (homing-required) is exactly the state `$H` exists to clear; the other (locked) alarms are
      // not — they demand a reset before any cycle.
      ControlState::Alarm(code) => matches!(code, AlarmCode::HomingRequired),
      _ => false,
    }
  }

  /// Apply a successful `$H` homing cycle (DOC-06): machine position is now established, so the machine returns
  /// to [`Normal`](ControlState::Normal) — clearing the homing-required boot alarm (`ALARM:11`). Only valid from
  /// a state [`homing_allowed`](ControlState::homing_allowed) cleared; from any other state it is a no-op
  /// (returns `self`), which is purely defensive since the consumer gates `$H` on `homing_allowed` first.
  pub fn home_complete(self) -> Self {
    if self.homing_allowed() {
      ControlState::Normal
    } else {
      self
    }
  }
}

/// The outcome of a `$X` unlock attempt, so the caller can choose the right response without re-inspecting
/// the state: emit `[MSG:Caution: Unlocked]` + `ok` on [`Unlocked`](UnlockOutcome::Unlocked), a bare `ok` on
/// [`NotAlarmed`](UnlockOutcome::NotAlarmed) (a no-op from a non-alarm state), or `error:N` on
/// [`Locked`](UnlockOutcome::Locked) (a locked critical alarm that only a soft reset can clear).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum UnlockOutcome {
  /// A non-locked alarm was cleared; emit `[MSG:Caution: Unlocked]` then `ok`.
  Unlocked,
  /// The machine was not in an alarm; `$X` is a no-op `ok`.
  NotAlarmed,
  /// A locked critical alarm (hard/soft limit, e-stop) — `$X` cannot clear it; reject with `error:N`.
  Locked,
}

/// The outcome of a `$C` check-mode toggle, so the caller emits the right `[MSG:...]` and runs the
/// soft-reset side-effects only when check mode is being disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CheckToggle {
  /// Check mode was entered; emit `[MSG:Enabled]` then `ok`.
  Enabled,
  /// Check mode was left; grbl follows this with a soft reset — emit `[MSG:Disabled]`, then run the reset
  /// side-effects (rebuild parser/planner, emit the banner) and `ok`.
  Disabled,
  /// The toggle was rejected (not in Normal/Check); the caller responds `error:N`.
  Rejected,
}
