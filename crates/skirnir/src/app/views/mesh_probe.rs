//! The height-map (mesh) acquisition panel, grid preview, and bench params.

use super::*;

/// Render the height-map acquisition panel (Part B2/B3). When no run is active it collects the grid bounds +
/// spacing (showing the derived point count), the grid-probe bench params, a Start button, and a Clear for a saved
/// mesh; while running it shows the serpentine progress and a Probe-next / Cancel pair. The acquisition state is
/// owned by the shell (the probed mesh is host state persisted to the profile), passed as a borrow, so this stays
/// a pure render that only emits intents.
pub fn mesh_probe(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState,
  run: Option<&crate::app::autolevel::MeshProbeState>, has_saved_mesh: bool, sink: &mut IntentSink) {
  use crate::app::autolevel::MeshProbeStep;
  let palette = state.style.palette;
  right_panel(ui, |ui| {
    let idle = view.connection == ConnectionState::Idle;
    let Some(w) = run else {
      // No run: collect the grid bounds + spacing and offer Start (and a Clear for a persisted mesh).
      dim_label(ui, palette, crate::tr!("mesh-intro"));
      ui.add_space(4.0);
      egui::Grid::new("mesh_setup").num_columns(3).show(ui, |ui| {
        ui.label(crate::tr!("lbl-mesh-min"));
        ui.add(egui::DragValue::new(&mut state.mesh_min.0).speed(0.5).suffix(" X"));
        ui.add(egui::DragValue::new(&mut state.mesh_min.1).speed(0.5).suffix(" Y"));
        ui.end_row();
        ui.label(crate::tr!("lbl-mesh-max"));
        ui.add(egui::DragValue::new(&mut state.mesh_max.0).speed(0.5).suffix(" X"));
        ui.add(egui::DragValue::new(&mut state.mesh_max.1).speed(0.5).suffix(" Y"));
        ui.end_row();
        ui.label(crate::tr!("lbl-mesh-spacing"));
        ui.add(egui::DragValue::new(&mut state.mesh_spacing).speed(0.5).range(0.1..=1000.0).suffix(" mm"));
        ui.end_row();
      });
      // Auto-fill the bounds from the loaded program's XY extents (the toolpath-bounds scan). Read the bounds into
      // an owned Option first so the immutable borrow is released before the click mutates the min/max fields.
      let program_bounds = state.program_xy_bounds();
      if gated_action_button(ui, program_bounds.is_some(), crate::tr!("btn-mesh-auto-bounds")).clicked()
        && let Some((min, max)) = program_bounds
      {
        state.mesh_min = min;
        state.mesh_max = max;
      }
      // The derived grid size (ceil(range/spacing)+1, min 2 per axis) so the operator sees the point count before
      // committing — this is exactly what `Mesh::from_spacing` will build.
      let (nx, ny) = grid_point_counts(state.mesh_min, state.mesh_max, state.mesh_spacing);
      dim_label(ui, palette, crate::tr!("mesh-grid-size", { nx: nx.to_string(), ny: ny.to_string() }));
      mesh_bench_params(ui, state);
      // Correction option (affects the STREAM, not acquisition): whether autolevel Z-corrects G0 rapids. Emits an
      // intent on change so the shell invalidates the corrected-program cache.
      let mut correct_rapids = state.autolevel_cfg.correct_rapids;
      if ui.checkbox(&mut correct_rapids, RichText::new(crate::tr!("mesh-correct-rapids")).size(11.0)).changed() {
        sink.push(Intent::SetCorrectRapids(correct_rapids));
      }
      ui.add_space(4.0);
      if gated_action_button(ui, idle, crate::tr!("btn-mesh-start")).clicked() {
        sink.push(Intent::MeshProbeStart {
          params: state.mesh_bench,
          min: state.mesh_min,
          max: state.mesh_max,
          spacing: (state.mesh_spacing, state.mesh_spacing),
        });
      }
      if has_saved_mesh {
        ui.add_space(4.0);
        ui.label(RichText::new(crate::tr!("mesh-saved")).size(11.0).color(palette.state_run));
        // Apply the saved mesh (arm autolevel against it) or clear it.
        if full_width_button(ui, crate::tr!("btn-mesh-apply")).clicked() {
          sink.push(Intent::ApplySavedMesh);
        }
        ui.add_space(2.0);
        if full_width_button(ui, crate::tr!("btn-mesh-clear")).clicked() {
          sink.push(Intent::MeshClear);
        }
      }
      return;
    };

    // A run is active: show progress, the live grid/Z preview, then the step's action + Cancel.
    let (done, total) = w.progress();
    let fraction = if total == 0 { 0.0 } else { done as f32 / total as f32 };
    ui.add(egui::ProgressBar::new(fraction).text(crate::tr!("mesh-progress", { done: done.to_string(), total: total.to_string() })));
    ui.add_space(4.0);
    mesh_preview(ui, palette, w);
    ui.add_space(6.0);
    let probing = w.is_probing();
    match w.step {
      MeshProbeStep::Ready => {
        dim_label(ui, palette, crate::tr!("mesh-step-ready"));
        if gated_action_button(ui, idle && !probing, crate::tr!("btn-mesh-probe")).clicked() {
          sink.push(Intent::MeshProbeNext);
        }
      }
      MeshProbeStep::Probing => {
        dim_label(ui, palette, crate::tr!("msg-probing-awaiting-dot"));
      }
      MeshProbeStep::Done => {
        ui.label(RichText::new(crate::tr!("mesh-done", { n: total.to_string() })).size(11.0).color(palette.state_run));
      }
      MeshProbeStep::Aborted => {
        let reason = w.abort_reason.clone().unwrap_or_else(|| crate::tr!("reason-cancelled"));
        ui.label(RichText::new(crate::tr!("msg-aborted", { reason: reason })).size(11.0).color(palette.state_alarm));
      }
    }
    ui.add_space(4.0);
    if full_width_button(ui, crate::tr!("btn-cancel-wizard")).clicked() {
      sink.push(Intent::MeshProbeCancel);
    }
  });
}

/// The derived `(nx, ny)` node counts for a grid `[min, max]` at `spacing`, matching
/// [`crate::app::autolevel::Mesh::from_spacing`] (`ceil(range/spacing)+1`, min 2; a non-positive spacing / zero range →
/// 2). Kept in the view so the panel can show the point count before a run without building a mesh.
fn grid_point_counts(min: (f64, f64), max: (f64, f64), spacing: f64) -> (usize, usize) {
  let count = |lo: f64, hi: f64| -> usize {
    let range = hi - lo;
    if spacing <= 0.0 || range <= 0.0 { 2 } else { ((range / spacing).ceil() as usize + 1).max(2) }
  };
  (count(min.0, max.0), count(min.1, max.1))
}

/// Render a simple live grid/Z preview of the acquisition: an `nx×ny` dot grid (Y up), each PROBED node shaded on
/// a cool→warm ramp by its Z delta (relative to the probed min/max), not-yet-probed nodes dim. A thin render of
/// the pure [`crate::app::autolevel::MeshProbeState`] — no state mutation.
fn mesh_preview(ui: &mut egui::Ui, palette: Palette, w: &crate::app::autolevel::MeshProbeState) {
  let mesh = w.mesh();
  let (nx, ny) = (mesh.nx, mesh.ny);
  if nx == 0 || ny == 0 {
    return;
  }
  let width = ui.available_width();
  let height = 72.0;
  let (rect, _) = ui.allocate_exact_size(Vec2::new(width, height), egui::Sense::hover());
  let painter = ui.painter_at(rect);
  // The probed set + the delta range over probed nodes, for the colour ramp.
  let probed: std::collections::HashSet<(usize, usize)> = w.probed().iter().copied().collect();
  let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
  for &(ix, iy) in w.probed() {
    let d = mesh.z[mesh.index(ix, iy)];
    lo = lo.min(d);
    hi = hi.max(d);
  }
  let span = (hi - lo).max(1e-6);
  let pad = 8.0;
  let cell_w = if nx > 1 { (width - 2.0 * pad) / (nx - 1) as f32 } else { 0.0 };
  let cell_h = if ny > 1 { (height - 2.0 * pad) / (ny - 1) as f32 } else { 0.0 };
  // Cool (low) → warm (high) ramp for probed nodes; dim for not-yet-probed.
  let cool = egui::Color32::from_rgb(60, 120, 220);
  let warm = egui::Color32::from_rgb(220, 90, 70);
  for iy in 0..ny {
    for ix in 0..nx {
      let cx = rect.left() + pad + ix as f32 * cell_w;
      let cy = rect.bottom() - pad - iy as f32 * cell_h; // flip Y so +Y is up, like the bed.
      let color = if probed.contains(&(ix, iy)) {
        let t = ((mesh.z[mesh.index(ix, iy)] - lo) / span) as f32;
        lerp_color(cool, warm, t)
      } else {
        palette.text_disabled
      };
      painter.circle_filled(egui::pos2(cx, cy), 3.0, color);
    }
  }
}

/// Linearly interpolate between two colours (per-channel), clamping `t` to `0..=1`.
fn lerp_color(a: egui::Color32, b: egui::Color32, t: f32) -> egui::Color32 {
  let t = t.clamp(0.0, 1.0);
  let mix = |x: u8, y: u8| (x as f32 + (y as f32 - x as f32) * t) as u8;
  egui::Color32::from_rgb(mix(a.r(), b.r()), mix(a.g(), b.g()), mix(a.b(), b.b()))
}

/// Render the editable grid-probe bench params in a collapsing section, mutating `state.mesh_bench` in place.
fn mesh_bench_params(ui: &mut egui::Ui, state: &mut UiState) {
  let palette = state.style.palette;
  let p = &mut state.mesh_bench;
  egui::CollapsingHeader::new(RichText::new(crate::tr!("hdr-mesh-bench")).size(11.0).color(palette.text_dim))
    .id_salt("mesh_bench_params")
    .show(ui, |ui| {
      egui::Grid::new("mesh_bench").num_columns(2).show(ui, |ui| {
        param_row(ui, crate::tr!("lbl-mesh-clearance"), &mut p.clearance_z, 0.1, -300.0..=0.0, " mm");
        param_row(ui, crate::tr!("lbl-mesh-feed"), &mut p.probe_feed, 5.0, 1.0..=5000.0, " mm/min");
        param_row(ui, crate::tr!("lbl-mesh-depth"), &mut p.probe_depth, 0.5, 0.1..=200.0, " mm");
      });
    });
}
