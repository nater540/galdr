//! The 2-D canvas: a pure pan/zoom world↔screen transform ([`CanvasView`], unit-tested without a window) and
//! the paint pass that draws the cached [`RenderScene`] with it.
//!
//! World space is engine millimetres, **Y up** (machine/board convention); screen space is egui points, Y down
//! — the transform owns the flip so nothing else ever thinks about it. Zoom anchors under the cursor: the
//! world point under the pointer stays fixed through a wheel step, which is what makes zooming into a pad feel
//! mechanical rather than drifty.

use eframe::egui::{self, Color32, Pos2, Rect, Stroke, vec2};

use super::intent::{Intent, IntentSink};
use super::scene::{Bounds, RenderScene};
use super::theme::Palette;
use super::views::UiState;
use crate::config::CanvasStyle;
use crate::tr;
use eitri_project::{ObjectId, ObjectKind};

/// The zoom bounds (px per mm): far enough out for a metre of travel, close enough in for a 0.1 mm trace.
const ZOOM_RANGE: std::ops::RangeInclusive<f32> = 0.05..=5000.0;

/// The pan/zoom state: which world point sits at the viewport centre, and the scale.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CanvasView {
  /// The world coordinate (mm) rendered at the centre of the viewport.
  pub center: [f64; 2],
  /// The scale, in screen points per millimetre.
  pub px_per_mm: f32,
}

impl Default for CanvasView {
  fn default() -> Self {
    // A fresh canvas frames roughly a 100 mm board in an 800 px viewport, centred a touch above the origin so
    // the crosshair is visible without anything loaded.
    CanvasView { center: [40.0, 30.0], px_per_mm: 8.0 }
  }
}

impl CanvasView {
  /// World (mm, Y-up) → screen (points, Y-down) within `rect`.
  pub fn to_screen(&self, world: [f64; 2], rect: Rect) -> Pos2 {
    let scale = self.px_per_mm as f64;
    let x = rect.center().x as f64 + (world[0] - self.center[0]) * scale;
    let y = rect.center().y as f64 - (world[1] - self.center[1]) * scale;
    Pos2::new(x as f32, y as f32)
  }

  /// Screen (points) → world (mm, Y-up) within `rect` — the inverse of [`Self::to_screen`].
  pub fn to_world(&self, pos: Pos2, rect: Rect) -> [f64; 2] {
    let scale = self.px_per_mm as f64;
    let x = self.center[0] + (pos.x - rect.center().x) as f64 / scale;
    let y = self.center[1] - (pos.y - rect.center().y) as f64 / scale;
    [x, y]
  }

  /// Pan by a screen-space drag delta (the world moves WITH the pointer).
  pub fn pan(&mut self, delta: egui::Vec2) {
    let scale = self.px_per_mm as f64;
    self.center[0] -= delta.x as f64 / scale;
    self.center[1] += delta.y as f64 / scale;
  }

  /// Zoom by `factor` keeping the world point under `anchor` fixed on screen. The scale is clamped to
  /// [`ZOOM_RANGE`]; a clamped step still keeps the anchor fixed (the effective factor is recomputed).
  pub fn zoom_about(&mut self, factor: f32, anchor: Pos2, rect: Rect) {
    let anchored = self.to_world(anchor, rect);
    self.px_per_mm = (self.px_per_mm * factor).clamp(*ZOOM_RANGE.start(), *ZOOM_RANGE.end());
    // Re-place the centre so `anchored` maps back to `anchor` at the new scale.
    let scale = self.px_per_mm as f64;
    self.center[0] = anchored[0] - (anchor.x - rect.center().x) as f64 / scale;
    self.center[1] = anchored[1] + (anchor.y - rect.center().y) as f64 / scale;
  }

  /// Frame `bounds` in `rect` with ~7% margin on the tight axis, centred. Degenerate bounds (a single point,
  /// an empty rect) fall back to a sane fixed scale rather than dividing by zero.
  pub fn fit(&mut self, bounds: Bounds, rect: Rect) {
    let (x0, y0, x1, y1) = bounds;
    self.center = [(x0 + x1) / 2.0, (y0 + y1) / 2.0];
    let (w_mm, h_mm) = ((x1 - x0).abs(), (y1 - y0).abs());
    if w_mm > f64::EPSILON && h_mm > f64::EPSILON && rect.width() > 1.0 && rect.height() > 1.0 {
      let fit_x = rect.width() as f64 * 0.93 / w_mm;
      let fit_y = rect.height() as f64 * 0.93 / h_mm;
      self.px_per_mm = (fit_x.min(fit_y) as f32).clamp(*ZOOM_RANGE.start(), *ZOOM_RANGE.end());
    } else {
      self.px_per_mm = 8.0;
    }
  }
}

/// Paint the canvas into `rect` and handle its pan/zoom/click input. A click picks the topmost visible object
/// under the pointer (via [`hit_object`]) and emits a [`Intent::Select`]; a click on empty canvas clears the
/// selection. Returns the world position under the pointer (for the status bar readout) when the pointer is
/// over the canvas.
pub fn show(ui: &mut egui::Ui, rect: Rect, scene: &RenderScene, state: &mut UiState, selected: Option<ObjectId>,
  sink: &mut IntentSink) -> Option<[f64; 2]> {
  let palette = state.style.palette;
  let style = state.style.canvas;
  // A queued fit request (the toolbar's Fit, or the auto-fit after a first open) is consumed here, where the
  // real viewport rect is known — intents never guess at layout.
  if state.pending_fit {
    if let Some(bounds) = scene.bounds {
      state.canvas.fit(bounds, rect);
    }
    state.pending_fit = false;
  }
  let painter = ui.painter_at(rect);
  painter.rect_filled(rect, 0.0, palette.inset);

  // Input first, so this frame already paints with the updated transform (no one-frame pan lag).
  let response = ui.interact(rect, ui.id().with("canvas"), egui::Sense::click_and_drag());
  response.widget_info(|| egui::WidgetInfo::labeled(egui::WidgetType::Other, ui.is_enabled(), tr!("canvas-label")));
  if response.dragged() {
    state.canvas.pan(response.drag_delta());
  }
  let hover_world = response.hover_pos().map(|pos| state.canvas.to_world(pos, rect));
  // Click-select: a real click (egui already excludes drags) picks the topmost object under the pointer, or
  // clears the selection on empty canvas. The tolerance is fixed in SCREEN px so picking a hairline trail
  // feels the same at every zoom.
  if response.clicked()
    && let Some(pos) = response.interact_pointer_pos()
  {
    let world = state.canvas.to_world(pos, rect);
    let tol_mm = (PICK_TOLERANCE_PX / state.canvas.px_per_mm.max(f32::EPSILON)) as f64;
    let hit = hit_object(scene, world, tol_mm);
    if hit != selected {
      sink.push(Intent::Select(hit));
    }
  }
  if let Some(pos) = response.hover_pos() {
    let scroll = ui.input(|i| i.smooth_scroll_delta.y);
    if scroll.abs() > 0.1 {
      // ~10% per notch-ish step; exponential so equal wheel travel is equal zoom ratio.
      state.canvas.zoom_about((scroll * 0.005).exp(), pos, rect);
    }
  }

  grid(&painter, rect, &state.canvas, palette, style);
  origin_cross(&painter, rect, &state.canvas, palette);

  for object in &scene.objects {
    // A hidden object contributes nothing to the paint pass (nor to picking, below).
    if !object.visible {
      continue;
    }
    // Fill pass: copper/drills/geometry as meshes, colour by kind. The opacity keeps trails readable over it.
    if object.fill.triangle_count() > 0 {
      let color = match object.kind {
        ObjectKind::Gerber => with_opacity(palette.copper, style.copper_opacity),
        ObjectKind::Excellon => with_opacity(palette.drill, style.copper_opacity),
        _ => with_opacity(palette.geometry, style.copper_opacity * 0.6),
      };
      let mut mesh = egui::Mesh::default();
      for v in &object.fill.vertices {
        mesh.colored_vertex(state.canvas.to_screen([v[0], v[1]], rect), color);
      }
      mesh.indices.extend_from_slice(&object.fill.indices);
      painter.add(egui::Shape::mesh(mesh));
    }
    // Outline pass: crisp rims over the fills.
    let edge = match object.kind {
      ObjectKind::Gerber => palette.copper_edge,
      ObjectKind::Excellon => palette.drill,
      _ => palette.geometry,
    };
    let edge_stroke = Stroke::new(style.outline_stroke_px, edge);
    for ring in &object.outlines {
      stroke_polyline(&painter, rect, &state.canvas, ring, edge_stroke);
    }
    for line in &object.polylines {
      stroke_polyline(&painter, rect, &state.canvas, line, Stroke::new(style.outline_stroke_px, palette.geometry));
    }
    // Toolpath pass: rapids under cuts, so the cut contour reads on top.
    for (from, to) in &object.rapids {
      painter.line_segment(
        [state.canvas.to_screen(*from, rect), state.canvas.to_screen(*to, rect)],
        Stroke::new(style.rapid_stroke_px, with_opacity(palette.toolpath_rapid, 0.55)),
      );
    }
    for cut in &object.cuts {
      stroke_polyline(&painter, rect, &state.canvas, cut, Stroke::new(style.cut_stroke_px, palette.toolpath_cut));
    }
    // Selection rim: a dashed-feeling thin rect around the selected object's bounds (paired with the tree
    // highlight, so selection is never colour-alone in one place).
    if selected == Some(object.id)
      && let Some((x0, y0, x1, y1)) = object.bounds
    {
      let a = state.canvas.to_screen([x0, y1], rect); // top-left on screen (y1 is the top in world space)
      let b = state.canvas.to_screen([x1, y0], rect);
      painter.rect_stroke(
        Rect::from_two_pos(a, b).expand(4.0),
        2.0,
        Stroke::new(1.0, palette.selection),
        egui::StrokeKind::Outside,
      );
    }
  }

  if scene.objects.is_empty() {
    empty_state(&painter, rect, palette);
  }
  hover_world
}

/// The screen-space picking tolerance for click-select (points): how far a click may land from a stroked
/// polyline (an outline ring, an imported stroke, a cut trail) and still pick its object.
pub const PICK_TOLERANCE_PX: f32 = 6.0;

/// Pick the topmost visible object at `world` (mm): objects are tested in REVERSE display order (painted
/// back-to-front, so the last is on top). A hit is a point inside a fill triangle, or within `tol_mm` of an
/// outline ring, an open polyline, or a cut trail. Rapids are travel context, never pickable. Pure — unit
/// tested without a window; the paint pass calls it with [`PICK_TOLERANCE_PX`] converted through the zoom.
pub fn hit_object(scene: &RenderScene, world: [f64; 2], tol_mm: f64) -> Option<ObjectId> {
  let tol_sq = tol_mm * tol_mm;
  for object in scene.objects.iter().rev() {
    if !object.visible {
      continue;
    }
    // Cheap reject: outside the object's tolerance-expanded bounds nothing below can hit.
    let Some((x0, y0, x1, y1)) = object.bounds else { continue };
    if world[0] < x0 - tol_mm || world[0] > x1 + tol_mm || world[1] < y0 - tol_mm || world[1] > y1 + tol_mm {
      continue;
    }
    let mesh = &object.fill;
    let inside_fill = mesh.indices.chunks_exact(3).any(|tri| {
      let (a, b, c) = (mesh.vertices[tri[0] as usize], mesh.vertices[tri[1] as usize], mesh.vertices[tri[2] as usize]);
      point_in_triangle(world, a, b, c)
    });
    if inside_fill {
      return Some(object.id);
    }
    let near_stroke = object
      .outlines
      .iter()
      .chain(&object.polylines)
      .chain(&object.cuts)
      .any(|line| line.windows(2).any(|seg| dist_sq_point_segment(world, seg[0], seg[1]) <= tol_sq));
    if near_stroke {
      return Some(object.id);
    }
  }
  None
}

/// Whether `p` lies inside (or on the edge of) triangle `abc`, via consistent cross-product signs.
fn point_in_triangle(p: [f64; 2], a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> bool {
  let cross = |o: [f64; 2], u: [f64; 2], v: [f64; 2]| (u[0] - o[0]) * (v[1] - o[1]) - (u[1] - o[1]) * (v[0] - o[0]);
  let (d1, d2, d3) = (cross(a, b, p), cross(b, c, p), cross(c, a, p));
  let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
  let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
  !(has_neg && has_pos)
}

/// Squared distance from `p` to segment `ab`.
fn dist_sq_point_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
  let (abx, aby) = (b[0] - a[0], b[1] - a[1]);
  let len_sq = abx * abx + aby * aby;
  let t = if len_sq <= f64::EPSILON {
    0.0
  } else {
    (((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / len_sq).clamp(0.0, 1.0)
  };
  let (dx, dy) = (p[0] - (a[0] + t * abx), p[1] - (a[1] + t * aby));
  dx * dx + dy * dy
}

/// Stroke one polyline through the transform.
fn stroke_polyline(painter: &egui::Painter, rect: Rect, view: &CanvasView, pts: &[[f64; 2]], stroke: Stroke) {
  if pts.len() < 2 {
    return;
  }
  let screen: Vec<Pos2> = pts.iter().map(|p| view.to_screen(*p, rect)).collect();
  painter.add(egui::Shape::line(screen, stroke));
}

/// The adaptive millimetre grid: the minor step is the smallest 1/2/5×10ⁿ mm whose screen spacing clears the
/// configured minimum, majors every Nth minor. Anchored to world zero so the grid never swims under pan.
fn grid(painter: &egui::Painter, rect: Rect, view: &CanvasView, palette: Palette, style: CanvasStyle) {
  let min_px = style.grid_minor_px.max(2.0);
  let mut step_mm = 10f64.powf(((min_px / view.px_per_mm) as f64).log10().floor());
  for factor in [1.0, 2.0, 5.0, 10.0] {
    if (step_mm * factor) * view.px_per_mm as f64 >= min_px as f64 {
      step_mm *= factor;
      break;
    }
  }
  if !(step_mm.is_finite() && step_mm > 0.0) {
    return;
  }
  let major_every = style.grid_major_every.max(1) as i64;
  let top_left = view.to_world(rect.min, rect);
  let bottom_right = view.to_world(rect.max, rect);
  let (x_first, x_last) = ((top_left[0] / step_mm).floor() as i64, (bottom_right[0] / step_mm).ceil() as i64);
  let (y_first, y_last) = ((bottom_right[1] / step_mm).floor() as i64, (top_left[1] / step_mm).ceil() as i64);
  // A hard cap on line count keeps a degenerate zoom state from painting tens of thousands of segments.
  if (x_last - x_first) + (y_last - y_first) > 4000 {
    return;
  }
  for ix in x_first..=x_last {
    let x = ix as f64 * step_mm;
    let color = if ix % major_every == 0 { palette.grid_major } else { palette.grid_minor };
    let sx = view.to_screen([x, 0.0], rect).x;
    painter.line_segment([Pos2::new(sx, rect.top()), Pos2::new(sx, rect.bottom())], Stroke::new(1.0, color));
  }
  for iy in y_first..=y_last {
    let y = iy as f64 * step_mm;
    let color = if iy % major_every == 0 { palette.grid_major } else { palette.grid_minor };
    let sy = view.to_screen([0.0, y], rect).y;
    painter.line_segment([Pos2::new(rect.left(), sy), Pos2::new(rect.right(), sy)], Stroke::new(1.0, color));
  }
}

/// The origin crosshair: a small violet cross at world (0, 0), the anchor every import lands relative to.
fn origin_cross(painter: &egui::Painter, rect: Rect, view: &CanvasView, palette: Palette) {
  let origin = view.to_screen([0.0, 0.0], rect);
  if !rect.expand(12.0).contains(origin) {
    return;
  }
  let stroke = Stroke::new(1.2, palette.origin);
  painter.line_segment([origin - vec2(8.0, 0.0), origin + vec2(8.0, 0.0)], stroke);
  painter.line_segment([origin - vec2(0.0, 8.0), origin + vec2(0.0, 8.0)], stroke);
}

/// The empty state: a quiet centred invitation, not a blank void.
fn empty_state(painter: &egui::Painter, rect: Rect, palette: Palette) {
  painter.text(
    rect.center() - vec2(0.0, 12.0),
    egui::Align2::CENTER_CENTER,
    tr!("canvas-empty-title"),
    egui::FontId::proportional(15.0),
    palette.text_dim,
  );
  painter.text(
    rect.center() + vec2(0.0, 10.0),
    egui::Align2::CENTER_CENTER,
    tr!("canvas-empty-hint"),
    egui::FontId::proportional(11.5),
    palette.text_disabled,
  );
}

/// Apply a `0..=1` opacity to an opaque palette colour at paint time (the palette stores opaque tokens).
fn with_opacity(color: Color32, opacity: f32) -> Color32 {
  color.gamma_multiply(opacity.clamp(0.0, 1.0))
}

#[cfg(test)]
mod tests {
  use super::*;

  fn rect() -> Rect {
    Rect::from_min_size(Pos2::new(100.0, 50.0), vec2(800.0, 600.0))
  }

  #[test]
  fn world_screen_round_trips_and_y_flips() {
    let view = CanvasView { center: [10.0, 20.0], px_per_mm: 4.0 };
    let r = rect();
    // The centre world point lands at the viewport centre.
    assert_eq!(view.to_screen([10.0, 20.0], r), r.center());
    // +Y in world goes UP on screen (smaller screen y).
    let up = view.to_screen([10.0, 25.0], r);
    assert!(up.y < r.center().y, "world +Y must render upward");
    assert_eq!(up.y, r.center().y - 5.0 * 4.0);
    // Round trip.
    for world in [[0.0, 0.0], [12.5, -3.25], [-40.0, 99.0]] {
      let back = view.to_world(view.to_screen(world, r), r);
      assert!((back[0] - world[0]).abs() < 1e-4 && (back[1] - world[1]).abs() < 1e-4, "{world:?} → {back:?}");
    }
  }

  #[test]
  fn pan_moves_the_world_with_the_pointer() {
    let mut view = CanvasView { center: [0.0, 0.0], px_per_mm: 2.0 };
    let r = rect();
    let before = view.to_screen([5.0, 5.0], r);
    view.pan(vec2(20.0, -10.0)); // drag right and up
    let after = view.to_screen([5.0, 5.0], r);
    assert!((after.x - (before.x + 20.0)).abs() < 1e-3, "content follows the drag in X");
    assert!((after.y - (before.y - 10.0)).abs() < 1e-3, "content follows the drag in Y");
  }

  #[test]
  fn zoom_keeps_the_anchored_world_point_fixed() {
    let mut view = CanvasView { center: [30.0, 40.0], px_per_mm: 3.0 };
    let r = rect();
    let anchor = Pos2::new(300.0, 200.0);
    let world_under_anchor = view.to_world(anchor, r);
    view.zoom_about(1.8, anchor, r);
    let after = view.to_screen(world_under_anchor, r);
    assert!((after.x - anchor.x).abs() < 1e-2 && (after.y - anchor.y).abs() < 1e-2,
      "the world point under the cursor must stay under it: {after:?} vs {anchor:?}");
    assert!((view.px_per_mm - 5.4).abs() < 1e-4);
  }

  #[test]
  fn zoom_clamps_to_sane_bounds_without_losing_the_anchor() {
    let mut view = CanvasView { center: [0.0, 0.0], px_per_mm: 1.0 };
    let r = rect();
    let anchor = Pos2::new(200.0, 100.0);
    let world = view.to_world(anchor, r);
    view.zoom_about(1e9, anchor, r);
    assert_eq!(view.px_per_mm, *ZOOM_RANGE.end(), "an absurd zoom-in clamps");
    let after = view.to_screen(world, r);
    assert!((after.x - anchor.x).abs() < 0.5 && (after.y - anchor.y).abs() < 0.5, "clamped zoom keeps the anchor");
    view.zoom_about(1e-12, anchor, r);
    assert_eq!(view.px_per_mm, *ZOOM_RANGE.start(), "an absurd zoom-out clamps too");
  }

  #[test]
  fn fit_frames_the_bounds_inside_the_viewport() {
    let mut view = CanvasView::default();
    let r = rect();
    view.fit((0.0, 0.0, 100.0, 50.0), r);
    assert_eq!(view.center, [50.0, 25.0], "fit centres the bounds");
    // Every corner must land inside the viewport with the margin.
    for corner in [[0.0, 0.0], [100.0, 0.0], [0.0, 50.0], [100.0, 50.0]] {
      let p = view.to_screen(corner, r);
      assert!(r.contains(p), "corner {corner:?} must be inside the viewport, got {p:?}");
    }
    // The tight axis fills most of the viewport (the margin is ~7%).
    let left = view.to_screen([0.0, 25.0], r).x;
    let right = view.to_screen([100.0, 25.0], r).x;
    assert!((right - left) > r.width() * 0.85, "fit should actually fill the viewport");
  }

  #[test]
  fn fit_of_degenerate_bounds_falls_back_rather_than_exploding() {
    let mut view = CanvasView::default();
    let r = rect();
    view.fit((5.0, 5.0, 5.0, 5.0), r); // a single point
    assert_eq!(view.center, [5.0, 5.0]);
    assert!(view.px_per_mm.is_finite() && view.px_per_mm > 0.0, "a point fit must not divide by zero");
  }

  // ── Click picking ────────────────────────────────────────────────────────────────────────────────────────

  use super::super::scene::{ObjectScene, RenderScene};
  use eitri_geo::TriangleMesh;

  /// A synthetic filled square `[0,10]²` as one object — two triangles, a square outline ring.
  fn filled_square(id: u64) -> ObjectScene {
    ObjectScene {
      id: ObjectId(id),
      kind: ObjectKind::Gerber,
      visible: true,
      fill: TriangleMesh {
        vertices: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]],
        indices: vec![0, 1, 2, 0, 2, 3],
      },
      outlines: vec![vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [0.0, 0.0]]],
      polylines: Vec::new(),
      cuts: Vec::new(),
      rapids: Vec::new(),
      bounds: Some((0.0, 0.0, 10.0, 10.0)),
    }
  }

  /// A synthetic toolpath object: one diagonal cut trail across the square, no fill.
  fn trail(id: u64) -> ObjectScene {
    ObjectScene {
      id: ObjectId(id),
      kind: ObjectKind::CncJob,
      visible: true,
      fill: TriangleMesh::default(),
      outlines: Vec::new(),
      polylines: Vec::new(),
      cuts: vec![vec![[0.0, 0.0], [10.0, 10.0]]],
      rapids: vec![([0.0, 10.0], [10.0, 0.0])],
      bounds: Some((0.0, 0.0, 10.0, 10.0)),
    }
  }

  fn scene_of(objects: Vec<ObjectScene>) -> RenderScene {
    let mut bounds: Option<super::super::scene::Bounds> = None;
    for o in &objects {
      if let (Some((x0, y0, x1, y1)), true) = (o.bounds, o.visible) {
        bounds = Some(match bounds {
          None => (x0, y0, x1, y1),
          Some((a, b, c, d)) => (x0.min(a), y0.min(b), x1.max(c), y1.max(d)),
        });
      }
    }
    RenderScene { objects, bounds }
  }

  #[test]
  fn a_point_inside_a_fill_hits_the_object_and_a_far_point_hits_nothing() {
    let scene = scene_of(vec![filled_square(1)]);
    assert_eq!(hit_object(&scene, [5.0, 5.0], 0.5), Some(ObjectId(1)), "inside the filled square");
    assert_eq!(hit_object(&scene, [50.0, 50.0], 0.5), None, "far away misses");
    assert_eq!(hit_object(&scene, [12.0, 5.0], 0.5), None, "just outside (beyond tolerance) misses");
  }

  #[test]
  fn a_point_near_a_cut_trail_hits_the_topmost_object_over_the_fill_below() {
    // The job is drawn AFTER the copper (later in display order = on top), so a click on its trail picks the
    // job even though the point is also inside the copper fill.
    let scene = scene_of(vec![filled_square(1), trail(2)]);
    assert_eq!(hit_object(&scene, [5.0, 5.2], 0.5), Some(ObjectId(2)), "the trail is topmost where they overlap");
    // Inside the copper but far from the diagonal: the copper wins.
    assert_eq!(hit_object(&scene, [8.0, 1.0], 0.5), Some(ObjectId(1)), "off the trail the fill is picked");
  }

  #[test]
  fn rapids_are_not_pickable_and_an_invisible_object_is_skipped() {
    // A point on the anti-diagonal rapid (but off the cut) must NOT pick the job — rapids are travel context,
    // not selectable geometry.
    let scene = scene_of(vec![filled_square(1), trail(2)]);
    assert_eq!(hit_object(&scene, [2.0, 8.0], 0.3), Some(ObjectId(1)), "a rapid never picks its job");

    let mut hidden = filled_square(1);
    hidden.visible = false;
    let scene = scene_of(vec![hidden]);
    assert_eq!(hit_object(&scene, [5.0, 5.0], 0.5), None, "a hidden object cannot be clicked");
  }

  #[test]
  fn open_polylines_pick_within_tolerance_only() {
    let mut geometry = filled_square(3);
    geometry.fill = TriangleMesh::default();
    geometry.outlines = Vec::new();
    geometry.polylines = vec![vec![[0.0, 0.0], [10.0, 0.0]]];
    let scene = scene_of(vec![geometry]);
    assert_eq!(hit_object(&scene, [5.0, 0.3], 0.5), Some(ObjectId(3)), "within tolerance of the stroke");
    assert_eq!(hit_object(&scene, [5.0, 1.2], 0.5), None, "beyond tolerance misses");
  }
}
