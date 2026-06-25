//! The GUI layer: the egui/eframe application that drives the streaming [`crate::engine::Engine`].
//!
//! The layer is split so the UI stays thin and the logic stays testable:
//!
//! - [`view_state`] — a pure, egui-free reducer that folds engine [`crate::engine::Event`]s into the
//!   [`view_state::ViewState`] the views render. All the decisions (DRO derivation, console capping, banner
//!   latching, progress) live here and are unit-tested without a window.
//! - [`intent`] — the one-way UI-intent vocabulary the views emit and the shell translates into engine
//!   commands and side effects. Also egui-free and host-testable.
//! - [`badge`] — pure, egui-free presentation logic: the machine-state [`badge::BadgeState`] derivation, the
//!   Run/Hold/Stop [`badge::TransportGroup`] enable/emphasis matrix, and the alarm/error detail copy. The
//!   [`theme`] turns a `BadgeState` into a colour; this module decides *which* state it is.
//!
//! The egui-touching pieces — the colour [`theme`], the per-panel [`views`], and the eframe [`shell`] — are
//! gated behind the `gui` feature so a headless build (CI, the engine/protocol/reducer tests) need not pull
//! egui/eframe at all.

pub mod angle_sweep;
pub mod badge;
pub mod flip_verify;
pub mod intent;
pub mod overrides;
pub mod preview;
pub mod probe_flow;
pub mod progress;
pub mod rotary_center;
pub mod rotary_probe;
pub mod runout;
pub mod setting_help;
pub mod settings_model;
pub mod settings_staging;
pub mod view_state;

pub use badge::{BadgeState, TransportGroup};
pub use intent::{Axis, Dir, Intent, IntentSink, work_offset_line, work_zero_line};
pub use overrides::{OverrideAxis, clamp_override, override_commands};
pub use progress::{TimeEstimate, estimate, format_mmss};
pub use setting_help::SettingDescriptions;
pub use settings_model::{SettingRow, SettingsModel};
pub use settings_staging::SettingsStaging;
pub use view_state::{Banner, CONSOLE_CAPACITY, LogLine, LogSource, Progress, ViewState};

#[cfg(feature = "gui")]
pub mod fonts;
#[cfg(feature = "gui")]
pub mod metrics;
#[cfg(feature = "gui")]
pub mod shell;
#[cfg(feature = "gui")]
pub mod theme;
#[cfg(feature = "gui")]
pub mod views;

/// The egui_kittest UI test harness, compiled only for the gui-featured test build. See the module docs for how
/// to add UI tests; it hosts the override-slider regression tests.
#[cfg(all(test, feature = "gui"))]
mod ui_test;

#[cfg(feature = "gui")]
pub use shell::{SkirnirApp, run};
#[cfg(feature = "gui")]
pub use theme::Theme;
