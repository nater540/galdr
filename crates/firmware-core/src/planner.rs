//! Motion planner (DOC-05).
//!
//! Mirrors grbl: computes only optimal per-block entry speeds via forward/reverse passes plus
//! junction-deviation cornering. Blocks flow to the motion executor through a ring buffer. The
//! trapezoidal profile itself is realized downstream by the segment generator in [`crate::motion`].
//!
//! Status: not yet implemented.
// TODO(DOC-05): implement the BlockQueue, junction-deviation cornering, and forward/reverse passes.
