//! The runout report (DOC-11 §2.2): measure stock runout / eccentricity. READ-ONLY — never writes a `G10`.
//!
//! Probe a feature at N evenly-spaced A angles around the part (same linear approach each time, only A changes),
//! collect the N radial readings, and report:
//! - **TIR (total indicator reading)** = `max − min` of the readings.
//! - **eccentricity** = `TIR / 2`.
//!
//! Pure math over a completed [`super::angle_sweep`]; the sweep does the probing/sequencing and the shell drives
//! it. Nothing here emits g-code — the report is informational only, so there is no `G10` offer and no axis-word
//! safety concern (the only motion is the sweep's own rotary-safe probes, which already never carry an A word in
//! the probe line).

/// The computed runout report over a set of radial readings (one per probed angle).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RunoutReport {
  /// The total indicator reading: `max − min` of the readings. Always ≥ 0.
  pub tir: f64,
  /// The eccentricity: `TIR / 2`.
  pub eccentricity: f64,
  /// How many readings the report was computed from.
  pub count: usize,
}

impl RunoutReport {
  /// Compute the report from the sweep's `readings`. Returns `None` for fewer than two readings — TIR is
  /// meaningless with zero or one sample, so the wizard shows nothing rather than a misleading `0.0`. `NaN`
  /// readings cannot occur (the `[PRB:]` parser rejects non-finite values), so a plain min/max fold is safe.
  pub fn from_readings(readings: &[f64]) -> Option<Self> {
    if readings.len() < 2 {
      return None;
    }
    let mut min = readings[0];
    let mut max = readings[0];
    for &r in &readings[1..] {
      if r < min {
        min = r;
      }
      if r > max {
        max = r;
      }
    }
    let tir = max - min;
    Some(RunoutReport { tir, eccentricity: tir / 2.0, count: readings.len() })
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn tir_is_max_minus_min_and_eccentricity_is_half() {
    // Readings spanning [-0.5, 1.5] → TIR = 2.0, eccentricity = 1.0.
    let r = RunoutReport::from_readings(&[0.0, 1.5, -0.5, 1.0]).expect("a report");
    assert_eq!(r.tir, 2.0);
    assert_eq!(r.eccentricity, 1.0);
    assert_eq!(r.count, 4);
  }

  #[test]
  fn perfectly_round_stock_reads_zero_runout() {
    let r = RunoutReport::from_readings(&[3.0, 3.0, 3.0, 3.0]).expect("a report");
    assert_eq!(r.tir, 0.0);
    assert_eq!(r.eccentricity, 0.0);
  }

  #[test]
  fn two_readings_are_enough() {
    let r = RunoutReport::from_readings(&[1.0, 4.0]).expect("a report");
    assert_eq!(r.tir, 3.0);
    assert_eq!(r.eccentricity, 1.5);
  }

  #[test]
  fn fewer_than_two_readings_yields_no_report() {
    // TIR is meaningless with 0 or 1 sample — show nothing rather than a misleading zero.
    assert_eq!(RunoutReport::from_readings(&[]), None);
    assert_eq!(RunoutReport::from_readings(&[5.0]), None);
  }

  #[test]
  fn handles_negative_readings() {
    let r = RunoutReport::from_readings(&[-3.0, -1.0, -2.0]).expect("a report");
    assert_eq!(r.tir, 2.0);
    assert_eq!(r.eccentricity, 1.0);
  }
}
