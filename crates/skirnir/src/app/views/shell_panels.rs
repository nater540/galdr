//! The right-hand panel column and the setup/probing menu + floating dialog windows.

use super::*;

/// Lay out the ENTIRE window-panel arrangement — toolbar, alarm/tool-change banner, status bar, bottom dock,
/// the fixed left/right columns, and the central toolpath viewport — into the window's root `Ui`. This is THE
/// single description of the app's frame: `SkirnirApp::ui` calls it with live shell state and the whole-window
/// test harness calls it with fixtures, so the two can never drift (the harness used to hand-copy this
/// arrangement, and the copy silently lost the tool-change banner branch — the failure mode this extraction
/// makes structurally impossible). The ctx-level floating windows (firmware settings, app settings, confirm
/// modals) are NOT part of the panel grid and stay with the shell.
pub fn shell_panels(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, data: ShellPanelsData<'_>,
  sink: &mut IntentSink) {
  let palette = state.style.palette;

  // The toolbar is a fixed 40px bar (design §03); pin it so it neither collapses nor grows with content. It
  // carries the `panelAlt` (#222222) surface — a shade lighter than the panels below — so the toolbar reads as
  // distinct chrome rather than blending into the body (the design's toolbar fill, previously the panel grey).
  egui::Panel::top("toolbar").exact_size(Metrics::TOOLBAR_H)
    .frame(egui::Frame::NONE.fill(palette.panel_alt))
    .show(ui, |ui| {
      toolbar(ui, view, state, sink);
    });

  if view.banner.is_some() {
    egui::Panel::top("banner").show(ui, |ui| {
      alarm_banner(ui, palette, view, sink);
    });
  } else if view.badge_state() == BadgeState::Tool {
    // No fault is latched, but the firmware is held for an M6 manual tool change: surface the prominent
    // tool-change affordance in the same top slot (a fault banner, if any, takes precedence above). The Resume
    // action routes through the existing cycle-start path, not a second control. The banner names the tool from
    // `view.current_tool` — the firmware answers `$G` during the hold (the shell nudges it on the transition).
    egui::Panel::top("tool_change").show(ui, |ui| {
      tool_change_banner(ui, palette, view, sink);
    });
  }

  // The status bar is a fixed 24px mono strip (design §03).
  egui::Panel::bottom("status").exact_size(Metrics::STATUS_BAR_H).show(ui, |ui| {
    status_bar(ui, view, state);
  });

  // COLLAPSED, the dock is just its tab strip: a fixed, hand-rolled bottom panel spanning the full window width
  // (exact-sized, so no resize machinery is in play). EXPANDED, the dock lives in the central egui_tiles split
  // below — so it is laid out AFTER the side columns and spans the viewport width, not the window width, and
  // the columns run the full height between toolbar and status bar.
  if state.dock_collapsed {
    egui::Panel::bottom("dock-collapsed").resizable(false).exact_size(Metrics::DOCK_COLLAPSED_H)
      .show(ui, |ui| {
        dock(ui, view, state, data.time, data.eta_qualifier, sink);
      });
  }

  // The design body grid is a fixed `268px | 1fr | 286px`: the left (DRO + Jog) and right (Overrides + Probe +
  // Settings) columns are exact widths, not resizable, so the layout matches the mock regardless of window
  // size. Program no longer lives in the right column — it is a dock tab now (design §03).
  //
  // Each panel is given a zero-inner-margin `Frame` (panel-filled) rather than egui's default side-panel frame
  // (`Margin::symmetric(8, 2)`). The default 8px L/R inset would shrink the usable column to 252px while the
  // section headers and DRO/Jog bodies already own their padding (`HEADER_PAD_X`, `DRO_PAD`, `JOG_PAD`), so the
  // content overran the clipped 252px and the rightmost controls ("Zero XYZ", the Z± column) were cut off. With
  // the margin zeroed the full 268/286 is usable and the views' own padding sets the gutters the design intends.
  let column_frame = egui::Frame::NONE.fill(palette.panel);
  egui::Panel::left("controls").resizable(false).exact_size(Metrics::LEFT_COL_W).frame(column_frame)
    .show(ui, |ui| contained(ui, |ui| {
      // `auto_shrink([false, false])` pins the content to the full 268px column instead of letting the scroll
      // area shrink to the widest child, which otherwise leaves an unfilled strip on the column's inner edge.
      egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        // Cap the content width to the column: a vertical scroll area REMEMBERS its content size, so a single
        // frame where a full-width allocation (the section header bars) reads a transiently wide
        // `available_width` ratchets the content wider forever — and under egui 0.35 a side panel whose content
        // overflows its width re-anchors its measured rect on the overflowed edge, shifting the central region
        // over the column (the clipped-labels bug). A hard cap makes the ratchet impossible.
        ui.set_max_width(Metrics::LEFT_COL_W);
        dro(ui, view, state, sink);
        ui.separator();
        jog(ui, view, state, sink);
      });
    }));

  egui::Panel::right("rightcol").resizable(false).exact_size(Metrics::RIGHT_COL_W).frame(column_frame)
    .show(ui, |ui| contained(ui, |ui| {
      // `auto_shrink([false, false])`: fill the full fixed column width and height so the content never
      // collapses to its natural size and leaves a bare strip beside it. Settings live only in the toolbar's
      // Settings window now, not as a right-column section.
      egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| {
        // Same content-width cap as the left column (see there): the ratchet bug manifested HERE first, as the
        // overrides header allocating a transiently-wide row and pushing the whole column off-window.
        ui.set_max_width(Metrics::RIGHT_COL_W);
        // Only the live overrides stay in the always-visible column — the thing an operator watches and adjusts
        // DURING a running job. The heavy setup wizards (probe/touch-off, rotary center, datum, height-map,
        // verify/measure) used to stack below here as a six-section scroll; they now live in dialogs launched from
        // the compact menu, so the column stays short and scannable.
        overrides(ui, view, state, sink);
        ui.separator();
        setup_menu(ui, state, SetupRunning {
          rotary: data.wizard.is_some(),
          datum: data.datum.is_some(),
          mesh: data.mesh_probe.is_some(),
          sweep: data.sweep.is_some(),
        });
      });
    }));

  // The central region takes a zero-margin frame (egui's default central-panel frame insets 8px on every side,
  // which left a black gutter beside the columns — the user-flagged band). Collapsed: the toolpath alone fills
  // it. Expanded: the egui_tiles [viewport | dock] split fills it — the split ratio is share-based state the
  // operator owns via the divider drag, and pane content structurally cannot feed back into it (the resizable-
  // panel self-resize bug class this replaced). The split is TAKEN OUT of the state for the render so the tiles
  // behavior can borrow the rest of the `UiState`, then put back with the (possibly dragged) fraction mirrored
  // for the profile to persist.
  egui::CentralPanel::default().frame(egui::Frame::NONE.fill(palette.inset)).show(ui, |ui| {
    if state.dock_collapsed {
      toolpath(ui, view, state);
    } else {
      let mut split = state
        .central_split
        .take()
        .unwrap_or_else(|| crate::app::dock_tiles::CentralSplit::new(state.dock_fraction));
      split.ui(ui, view, state, data.time, data.eta_qualifier, sink);
      state.dock_fraction = split.dock_fraction();
      state.central_split = Some(split);
    }
  });

  // A running probing flow FORCES its dialog visible (safety: a wizard that is moving the machine must never be
  // hidden behind a closed window); the operator's own open/closed choice is otherwise untouched. Then draw
  // whichever setup dialog is open as a floating window over the panel grid. Drawn here — inside the shared
  // `shell_panels`, from the shell's live wizard/datum/mesh/sweep borrows — so the whole-window test harness
  // drives the SAME dialogs the app does and the two cannot drift.
  if let Some(forced) = forced_setup_dialog(
    data.wizard.is_some(), data.datum.is_some(), data.mesh_probe.is_some(), data.sweep.is_some())
  {
    state.setup_dialog = Some(forced);
  }
  let ctx = ui.ctx().clone();
  setup_dialog_windows(&ctx, view, state, &data, sink);
}

/// Render the right column's compact "Setup &amp; probing" menu: a section header over a short stack of full-width
/// buttons, each OPENING one of the setup/probing dialogs rather than expanding a long inline panel (the six
/// stacked sections this replaced overran the column into a giant scroll). A button whose flow is running carries
/// a live "· running" tag — text, not colour alone — plus the running colour, so an in-progress wizard reads at a
/// glance even when its window sits behind another. Emits no machine intents; it only toggles
/// [`UiState::setup_dialog`].
pub(crate) fn setup_menu(ui: &mut egui::Ui, state: &mut UiState, running: SetupRunning) {
  let palette = state.style.palette;
  section_header(ui, palette, &crate::tr!("hdr-setup"));
  right_panel(ui, |ui| {
    dim_label(ui, palette, crate::tr!("setup-intro"));
    ui.add_space(8.0);
    // The five entries in workflow order: touch-off first, then the setup wizards. `hdr-*` keys double as both
    // the button label and the dialog title (see [`setup_dialog_windows`]).
    let entries = [
      (SetupDialog::Probe, crate::tr!("hdr-probe"), false),
      (SetupDialog::RotaryCenter, crate::tr!("hdr-rotary-center"), running.rotary),
      (SetupDialog::Datum, crate::tr!("hdr-datum"), running.datum),
      (SetupDialog::MeshProbe, crate::tr!("hdr-mesh"), running.mesh),
      (SetupDialog::VerifyMeasure, crate::tr!("hdr-verify"), running.sweep),
    ];
    for (index, (kind, label, is_running)) in entries.into_iter().enumerate() {
      if index > 0 {
        ui.add_space(4.0);
      }
      setup_menu_button(ui, palette, state, kind, &label, is_running);
    }
  });
}

/// One entry in the [`setup_menu`]: a full-width button toggling `kind`'s dialog. Shows a selected look while that
/// dialog is open and, when the flow is running, tints to the running colour and appends a "· running" tag (state
/// is never colour alone — accessibility). Truncates in a bounded width so a long translation can never overflow
/// the fixed column (the class of overflow that shifts the whole central region under egui 0.35).
fn setup_menu_button(ui: &mut egui::Ui, palette: Palette, state: &mut UiState, kind: SetupDialog, label: &str,
  running: bool) {
  let selected = state.setup_dialog == Some(kind);
  let text = if running {
    RichText::new(format!("{label}  ·  {}", crate::tr!("setup-running"))).size(11.5).color(palette.state_run)
  } else {
    RichText::new(label.to_string()).size(11.5)
  };
  let button = egui::Button::new(text).wrap_mode(egui::TextWrapMode::Truncate).selected(selected);
  let size = Vec2::new(ui.available_width(), Metrics::PANEL_CONTROL_H + 8.0);
  if ui.add_sized(size, button).clicked() {
    state.setup_dialog = toggle_setup_dialog(state.setup_dialog, kind);
  }
}

/// Draw whichever setup/probing dialog is open ([`UiState::setup_dialog`]) as a floating window over the panel
/// grid, hosting the existing panel render fn as its body. The window's default title bar is REPLACED by
/// [`setup_dialog_header`] so the floating wizard wears the app's own section-header treatment rather than
/// egui's mixed-case, bright window chrome — matching how the panels looked when they were inline. A running
/// flow's header omits the close `×` (and, being forced open every frame, cannot be dismissed mid-run) so an
/// in-progress wizard can only end from its in-body Cancel; an idle dialog's `×` clears the state. The window
/// is capped at the right column's width and scrolls vertically, so a tall wizard (datum with its bench params
/// expanded) never overflows the viewport.
fn setup_dialog_windows(ctx: &egui::Context, view: &ViewState, state: &mut UiState, data: &ShellPanelsData<'_>,
  sink: &mut IntentSink) {
  let Some(kind) = state.setup_dialog else {
    return;
  };
  let palette = state.style.palette;
  // A running flow pins its window open (no `×`); idle dialogs get a close button.
  let running = match kind {
    SetupDialog::Probe => false,
    SetupDialog::RotaryCenter => data.wizard.is_some(),
    SetupDialog::Datum => data.datum.is_some(),
    SetupDialog::MeshProbe => data.mesh_probe.is_some(),
    SetupDialog::VerifyMeasure => data.sweep.is_some(),
  };
  // A stable, translation-independent window id so egui remembers the operator's drag across a language switch
  // (the same reason the firmware settings window pins its id). The title is the translated section name.
  let (title, id) = match kind {
    SetupDialog::Probe => (crate::tr!("hdr-probe"), "setup-dialog-probe"),
    SetupDialog::RotaryCenter => (crate::tr!("hdr-rotary-center"), "setup-dialog-rotary"),
    SetupDialog::Datum => (crate::tr!("hdr-datum"), "setup-dialog-datum"),
    SetupDialog::MeshProbe => (crate::tr!("hdr-mesh"), "setup-dialog-mesh"),
    SetupDialog::VerifyMeasure => (crate::tr!("hdr-verify"), "setup-dialog-verify"),
  };
  // Custom chrome: a squared `panel`-filled frame (matching the inline column) with a 1px divider border and the
  // window shadow for float separation, and ZERO inner margin so the section-header strip and each body's own
  // `RIGHT_PAD` reach the edges exactly as they did inline. Dropping the title bar makes egui fall back to
  // drag-from-anywhere (see `WindowDrag`), so the dialog stays freely draggable without egui's chrome.
  let frame = egui::Frame::new()
    .fill(palette.panel)
    .stroke(egui::Stroke::new(Metrics::DIVIDER, palette.divider))
    .shadow(ctx.global_style().visuals.window_shadow)
    .inner_margin(0);
  let mut close_requested = false;
  // Open centred on the viewport (clear of the toolbar it otherwise covers at the default top-left placement);
  // still freely draggable, and egui remembers the operator's drag against the stable id thereafter.
  egui::Window::new(&title).id(egui::Id::new(id)).title_bar(false).frame(frame).collapsible(false)
    .resizable(false).default_width(Metrics::RIGHT_COL_W).max_width(Metrics::RIGHT_COL_W).vscroll(true)
    .pivot(egui::Align2::CENTER_CENTER).default_pos(ctx.content_rect().center())
    .show(ctx, |ui| {
      // The same content-width cap the fixed columns carry: the wizard rows allocate full-width controls, and a
      // transiently-wide allocation must not ratchet the window (or overflow it) in any locale.
      ui.set_max_width(Metrics::RIGHT_COL_W);
      // The app-styled title bar stands in for the section header the panels dropped when they moved here; a
      // running wizard's bar has no `×` (safety: a moving-machine flow cannot be hidden).
      if setup_dialog_header(ui, palette, &title, !running) {
        close_requested = true;
      }
      match kind {
        SetupDialog::Probe => probe(ui, view, state, sink),
        SetupDialog::RotaryCenter => rotary_center(ui, view, state, data.wizard, data.has_saved_center, sink),
        SetupDialog::Datum => datum_finder(ui, view, state, data.datum, sink),
        SetupDialog::MeshProbe => mesh_probe(ui, view, state, data.mesh_probe, data.has_saved_mesh, sink),
        SetupDialog::VerifyMeasure => verify_measure(ui, view, state, data.sweep, sink),
      }
    });
  if close_requested {
    state.setup_dialog = None;
  }
}
