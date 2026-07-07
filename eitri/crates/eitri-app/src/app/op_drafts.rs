//! The pure parameter drafts behind the CAM op panels (paint, non-copper clear, cutout, panelize, mirror,
//! film): plain numbers the operator owns until Run converts them into the engine's operator-facing specs.
//! Every conversion floors/clamps degenerate hand-typed values the same way `views::IsolationDraft` does —
//! the engine validates too; this keeps the obvious cases from ever leaving the panel. Egui-free and
//! unit-tested without a window.

use eitri_cam::{FilmKind, FilmParams};
use eitri_gcode::IsolationJob;
use eitri_project::{
  BoundarySpec, CutoutOutlineSpec, CutoutSpec, DirectionSpec, MirrorLineSpec, NonCopperSpec, ObjectId, PaintSpec,
  PaintStrategySpec, PanelizeSpec, SpacingSpec, TabPlacementSpec,
};

/// The shared emission drafts (depths, feeds, spindle) the toolpath-producing panels append below their own
/// parameters. One struct so paint/non-copper/cutout stay in step with each other.
#[derive(Debug, Clone, PartialEq)]
pub struct JobDraft {
  /// Total cut depth below the surface (positive mm).
  pub cut_depth: f64,
  /// Depth per pass (positive mm; 0 = single pass).
  pub pass_depth: f64,
  /// Cutting feed (mm/min).
  pub cut_feed: f64,
  /// Plunge feed (mm/min).
  pub plunge_feed: f64,
  /// Safe travel height (positive mm).
  pub travel_z: f64,
  /// Spindle speed (RPM).
  pub spindle_rpm: f64,
}

impl Default for JobDraft {
  fn default() -> Self {
    let job = IsolationJob::default();
    JobDraft {
      cut_depth: job.cut_depth,
      pass_depth: job.pass_depth,
      cut_feed: job.cut_feed,
      plunge_feed: job.plunge_feed,
      travel_z: job.travel_z,
      spindle_rpm: job.spindle_rpm,
    }
  }
}

impl JobDraft {
  /// The emission job, floored the same way as the isolation panel's.
  pub fn to_job(&self, name: Option<String>) -> IsolationJob {
    IsolationJob {
      cut_depth: self.cut_depth.max(0.001),
      pass_depth: self.pass_depth.max(0.0),
      cut_feed: self.cut_feed.max(1.0),
      plunge_feed: self.plunge_feed.max(1.0),
      travel_z: self.travel_z.max(0.1),
      spindle_rpm: self.spindle_rpm.max(0.0),
      name,
    }
  }
}

/// The paint fill pattern the panel offers (the raster angle lives beside it in the draft).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum StrategyChoice {
  /// Inward-offset rings, outer → inner.
  #[default]
  Concentric,
  /// The same rings inner → outer, growing from a seed.
  Seed,
  /// Parallel scan-line fill at the draft's angle.
  Raster,
}

/// The area-clearing (paint) parameter drafts.
#[derive(Debug, Clone, PartialEq)]
pub struct PaintDraft {
  /// Clearing tool diameter (mm).
  pub tool_diameter: f64,
  /// Pass overlap as a `0..=0.9` fraction of the tool diameter.
  pub overlap: f64,
  /// Inset from the region boundary before filling (mm).
  pub margin: f64,
  /// Climb (true) vs conventional milling.
  pub climb: bool,
  /// Append a boundary-following finishing pass.
  pub finish_pass: bool,
  /// The fill pattern.
  pub strategy: StrategyChoice,
  /// Scan-line angle (degrees), used when the strategy is raster.
  pub raster_angle: f64,
  /// The emission drafts.
  pub job: JobDraft,
}

impl Default for PaintDraft {
  fn default() -> Self {
    PaintDraft {
      tool_diameter: 1.0,
      overlap: 0.25,
      margin: 0.0,
      climb: true,
      finish_pass: true,
      strategy: StrategyChoice::Concentric,
      raster_angle: 0.0,
      job: JobDraft::default(),
    }
  }
}

impl PaintDraft {
  /// The engine spec, with degenerate values held to sane floors.
  pub fn to_spec(&self) -> PaintSpec {
    PaintSpec {
      tool_diameter: self.tool_diameter.max(0.01),
      overlap: self.overlap.clamp(0.0, 0.9),
      margin: self.margin.max(0.0),
      direction: if self.climb { DirectionSpec::Climb } else { DirectionSpec::Conventional },
      finish_pass: self.finish_pass,
      strategy: match self.strategy {
        StrategyChoice::Concentric => PaintStrategySpec::Concentric,
        StrategyChoice::Seed => PaintStrategySpec::Seed,
        StrategyChoice::Raster => PaintStrategySpec::Raster { angle_deg: self.raster_angle },
      },
    }
  }
}

/// How the non-copper clearing's outer frame is chosen in the panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum BoundaryChoice {
  /// The copper bounding box expanded by the draft's margin.
  #[default]
  BoundingBox,
  /// Another object's silhouette (resolved by the shell from the picked object's geometry).
  Object,
}

/// The non-copper-clearing parameter drafts: the boundary pick plus the paint pass that clears it.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct NonCopperDraft {
  /// How the outer frame is defined.
  pub boundary: BoundaryChoice,
  /// Bounding-box expansion (mm), for [`BoundaryChoice::BoundingBox`].
  pub boundary_margin: f64,
  /// The boundary object, for [`BoundaryChoice::Object`] (a Gerber/Geometry id the shell resolves).
  pub boundary_object: Option<ObjectId>,
  /// The clearing pass.
  pub paint: PaintDraft,
}

impl NonCopperDraft {
  /// The engine spec around an already-resolved boundary (the shell owns turning [`BoundaryChoice::Object`]
  /// into a concrete region; the draft cannot reach the session).
  pub fn to_spec(&self, boundary: BoundarySpec) -> NonCopperSpec {
    NonCopperSpec { boundary, paint: self.paint.to_spec() }
  }

  /// The bounding-box boundary this draft describes, margin floored at zero.
  pub fn bbox_boundary(&self) -> BoundarySpec {
    BoundarySpec::BoundingBox { margin: self.boundary_margin.max(0.0) }
  }
}

/// How the cutout outline is chosen in the panel.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum OutlineChoice {
  /// An axis-aligned rectangle from the draft's corners.
  #[default]
  Rectangle,
  /// The selected object's own silhouette (resolved by the shell).
  Silhouette,
}

/// The board-cutout parameter drafts.
#[derive(Debug, Clone, PartialEq)]
pub struct CutoutDraft {
  /// Routing tool diameter (mm).
  pub tool_diameter: f64,
  /// Width of the uncut gap left at each tab (mm).
  pub tab_width: f64,
  /// Number of evenly-spaced holding tabs per cut ring.
  pub tab_count: usize,
  /// Extra outward offset beyond the tool radius (mm).
  pub margin: f64,
  /// Climb (true) vs conventional milling.
  pub climb: bool,
  /// How the outline is defined.
  pub outline: OutlineChoice,
  /// Rectangle lower-left corner (mm), for [`OutlineChoice::Rectangle`].
  pub rect_min: [f64; 2],
  /// Rectangle upper-right corner (mm).
  pub rect_max: [f64; 2],
  /// The emission drafts.
  pub job: JobDraft,
}

impl Default for CutoutDraft {
  fn default() -> Self {
    CutoutDraft {
      tool_diameter: 2.0,
      tab_width: 3.0,
      tab_count: 4,
      margin: 0.0,
      climb: true,
      outline: OutlineChoice::Rectangle,
      rect_min: [0.0, 0.0],
      rect_max: [100.0, 80.0],
      job: JobDraft::default(),
    }
  }
}

impl CutoutDraft {
  /// The engine spec around an already-resolved outline (the shell owns [`OutlineChoice::Silhouette`]).
  pub fn to_spec(&self, outline: CutoutOutlineSpec) -> CutoutSpec {
    CutoutSpec {
      tool_diameter: self.tool_diameter.max(0.01),
      tab_width: self.tab_width.max(0.0),
      tabs: TabPlacementSpec::Count(self.tab_count),
      margin: self.margin.max(0.0),
      direction: if self.climb { DirectionSpec::Climb } else { DirectionSpec::Conventional },
      outline,
    }
  }

  /// The rectangle outline this draft describes, with swapped corners normalised so a hand-typed inverted
  /// rectangle still cuts what the operator drew.
  pub fn rectangle_outline(&self) -> CutoutOutlineSpec {
    let (x0, x1) = (self.rect_min[0].min(self.rect_max[0]), self.rect_min[0].max(self.rect_max[0]));
    let (y0, y1) = (self.rect_min[1].min(self.rect_max[1]), self.rect_min[1].max(self.rect_max[1]));
    CutoutOutlineSpec::Rectangle {
      min: geo_types::Coord { x: x0, y: y0 },
      max: geo_types::Coord { x: x1, y: y1 },
    }
  }
}

/// Whether the panel spacing values are clear gaps or centre-to-centre pitches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SpacingChoice {
  /// A clear band between copies.
  #[default]
  Gap,
  /// Centre-to-centre step.
  Pitch,
}

/// The panelization parameter drafts. Produces a **geometry** object, not a toolpath.
#[derive(Debug, Clone, PartialEq)]
pub struct PanelizeDraft {
  /// Rows (Y direction).
  pub rows: usize,
  /// Columns (X direction).
  pub cols: usize,
  /// Whether the spacing values are gaps or pitches (both axes share the mode).
  pub mode: SpacingChoice,
  /// Horizontal spacing (mm).
  pub x: f64,
  /// Vertical spacing (mm).
  pub y: f64,
}

impl Default for PanelizeDraft {
  fn default() -> Self {
    PanelizeDraft { rows: 2, cols: 2, mode: SpacingChoice::Gap, x: 5.0, y: 5.0 }
  }
}

impl PanelizeDraft {
  /// The engine spec: at least a 1×1 grid, spacing floored at zero.
  pub fn to_spec(&self) -> PanelizeSpec {
    let spacing = |v: f64| match self.mode {
      SpacingChoice::Gap => SpacingSpec::Gap(v.max(0.0)),
      SpacingChoice::Pitch => SpacingSpec::Pitch(v.max(0.0)),
    };
    PanelizeSpec { rows: self.rows.max(1), cols: self.cols.max(1), x: spacing(self.x), y: spacing(self.y) }
  }
}

/// The mirror axis the panel offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum MirrorAxisChoice {
  /// Reflect about `x = value` (a left-right flip).
  #[default]
  Vertical,
  /// Reflect about `y = value` (a top-bottom flip).
  Horizontal,
}

/// The two-sided mirror parameter drafts. Produces a **geometry** object.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct MirrorDraft {
  /// The axis to reflect about.
  pub axis: MirrorAxisChoice,
  /// The line's position (mm) on that axis.
  pub value: f64,
}

impl MirrorDraft {
  /// The engine mirror line.
  pub fn to_line(&self) -> MirrorLineSpec {
    match self.axis {
      MirrorAxisChoice::Vertical => MirrorLineSpec::Vertical(self.value),
      MirrorAxisChoice::Horizontal => MirrorLineSpec::Horizontal(self.value),
    }
  }
}

/// The film-export parameter drafts. Film is a vector SVG **export**, not a canvas op.
#[derive(Debug, Clone, PartialEq)]
pub struct FilmDraft {
  /// Negative (true) vs positive film.
  pub negative: bool,
  /// Uniform scale factor.
  pub scale: f64,
  /// Mirror horizontally (emulsion-side-down exposure).
  pub mirror: bool,
  /// Border around the artwork (mm).
  pub border: f64,
}

impl Default for FilmDraft {
  fn default() -> Self {
    let params = FilmParams::default();
    FilmDraft { negative: false, scale: params.scale, mirror: params.mirror, border: params.border }
  }
}

impl FilmDraft {
  /// The engine film parameters, scale floored positive and border floored at zero (the engine rejects the
  /// degenerate values outright; the floors keep a slipped drag from erroring at all).
  pub fn to_params(&self) -> FilmParams {
    FilmParams {
      kind: if self.negative { FilmKind::Negative } else { FilmKind::Positive },
      scale: self.scale.max(0.01),
      mirror: self.mirror,
      border: self.border.max(0.0),
    }
  }
}

#[cfg(test)]
mod tests {
  use super::*;

  #[test]
  fn paint_drafts_convert_with_floors_and_the_raster_angle_travels() {
    let draft = PaintDraft {
      tool_diameter: 0.0,
      overlap: 3.0,
      margin: -1.0,
      climb: false,
      strategy: StrategyChoice::Raster,
      raster_angle: 45.0,
      ..PaintDraft::default()
    };
    let spec = draft.to_spec();
    assert!(spec.tool_diameter > 0.0, "a zero tool diameter must be floored");
    assert!(spec.overlap <= 0.9 && spec.margin >= 0.0);
    assert_eq!(spec.direction, DirectionSpec::Conventional);
    assert_eq!(spec.strategy, PaintStrategySpec::Raster { angle_deg: 45.0 });

    let concentric = PaintDraft { strategy: StrategyChoice::Concentric, ..PaintDraft::default() }.to_spec();
    assert_eq!(concentric.strategy, PaintStrategySpec::Concentric);
    let seed = PaintDraft { strategy: StrategyChoice::Seed, ..PaintDraft::default() }.to_spec();
    assert_eq!(seed.strategy, PaintStrategySpec::Seed);
  }

  #[test]
  fn job_drafts_floor_degenerate_emission_values() {
    let job = JobDraft { cut_depth: -2.0, cut_feed: 0.0, plunge_feed: -1.0, travel_z: 0.0, ..JobDraft::default() }
      .to_job(Some("fixture".to_string()));
    assert!(job.cut_depth > 0.0 && job.cut_feed >= 1.0 && job.plunge_feed >= 1.0 && job.travel_z >= 0.1);
    assert_eq!(job.name.as_deref(), Some("fixture"));
  }

  #[test]
  fn noncopper_drafts_wrap_a_resolved_boundary_around_the_paint_pass() {
    let draft = NonCopperDraft { boundary_margin: -3.0, ..NonCopperDraft::default() };
    assert_eq!(draft.bbox_boundary(), BoundarySpec::BoundingBox { margin: 0.0 }, "the margin floors at zero");
    let spec = draft.to_spec(BoundarySpec::BoundingBox { margin: 2.0 });
    assert_eq!(spec.boundary, BoundarySpec::BoundingBox { margin: 2.0 });
    assert!(spec.paint.tool_diameter > 0.0);
  }

  #[test]
  fn cutout_drafts_normalise_an_inverted_rectangle_and_floor_the_tool() {
    let draft = CutoutDraft {
      tool_diameter: 0.0,
      tab_width: -1.0,
      tab_count: 3,
      rect_min: [50.0, 40.0],
      rect_max: [10.0, 5.0], // hand-typed inverted corners
      ..CutoutDraft::default()
    };
    let CutoutOutlineSpec::Rectangle { min, max } = draft.rectangle_outline() else {
      panic!("a rectangle draft yields a rectangle outline");
    };
    assert!(min.x < max.x && min.y < max.y, "swapped corners are normalised: {min:?} {max:?}");
    let spec = draft.to_spec(draft.rectangle_outline());
    assert!(spec.tool_diameter > 0.0 && spec.tab_width >= 0.0);
    assert_eq!(spec.tabs, TabPlacementSpec::Count(3));
  }

  #[test]
  fn panelize_drafts_floor_to_a_one_by_one_grid_and_map_both_spacing_modes() {
    let degenerate = PanelizeDraft { rows: 0, cols: 0, x: -2.0, y: -2.0, ..PanelizeDraft::default() }.to_spec();
    assert_eq!((degenerate.rows, degenerate.cols), (1, 1));
    assert_eq!(degenerate.x, SpacingSpec::Gap(0.0));

    let pitch = PanelizeDraft { mode: SpacingChoice::Pitch, x: 30.0, y: 25.0, ..PanelizeDraft::default() }.to_spec();
    assert_eq!(pitch.x, SpacingSpec::Pitch(30.0));
    assert_eq!(pitch.y, SpacingSpec::Pitch(25.0));
  }

  #[test]
  fn mirror_drafts_map_both_axes() {
    assert_eq!(
      MirrorDraft { axis: MirrorAxisChoice::Vertical, value: 12.5 }.to_line(),
      MirrorLineSpec::Vertical(12.5)
    );
    assert_eq!(
      MirrorDraft { axis: MirrorAxisChoice::Horizontal, value: -4.0 }.to_line(),
      MirrorLineSpec::Horizontal(-4.0)
    );
  }

  #[test]
  fn film_drafts_map_kind_and_floor_scale_and_border() {
    let draft = FilmDraft { negative: true, scale: 0.0, border: -2.0, mirror: true };
    let params = draft.to_params();
    assert_eq!(params.kind, FilmKind::Negative);
    assert!(params.scale > 0.0 && params.border >= 0.0);
    assert!(params.mirror);
    assert_eq!(FilmDraft::default().to_params().kind, FilmKind::Positive);
  }
}
