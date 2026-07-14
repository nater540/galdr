//! The dock body: the console log, the program listing, and the manual-command (MDI) strip.

use super::*;

/// The minimum width (px) the console's manual-command (MDI) text field is given when the row's available width,
/// after reserving the Send button, would otherwise leave it too narrow to see or type into. A floor so the field
/// can never collapse to an invisible sliver the way it did when the button consumed the whole row width first.
const MDI_FIELD_MIN_W: f32 = 120.0;

/// Render the Program tab body: the loaded file's lines with the acked line highlighted, drawn lazily so a
/// large program stays cheap to render. While streaming, the listing auto-scrolls to keep the executing line in
/// view (gated on the shared `auto-scroll` toggle), mirroring the console's stick-to-bottom follow.
pub(crate) fn program_body(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState) {
  let palette = state.style.palette;
  let row_height = ui.text_style_height(&egui::TextStyle::Monospace);
  let current = view.progress.acked;
  // Resolve the follow target before drawing so we can scroll the area to the executing row even when row
  // virtualisation has it off-screen (a `scroll_to_me` on the row only fires for rows actually in `range`).
  let follow = program_follow_target(
    state.auto_scroll,
    view.connection,
    current,
    state.program.len(),
    state.program_followed_line,
  );
  ScrollArea::vertical().auto_shrink([false, false]).show_rows(ui, row_height, state.program.len(), |ui, range| {
    // `show_rows` lays out only the visible slice, positioning the content `ui` so its top sits at the origin of
    // the FIRST visible row (`range.start`), not row 0 — and rows advance by `row_height + item_spacing.y`, not
    // the bare text height. Recover row 0's origin from the slice top, then any row's y is a flat multiple of the
    // full row pitch; this lets us scroll to a line virtualisation has not drawn this frame, which `scroll_to_me`
    // (row-local) cannot.
    if let Some(target) = follow {
      let row_pitch = row_height + ui.spacing().item_spacing.y;
      let row_zero_top = ui.min_rect().top() - range.start as f32 * row_pitch;
      let top = row_zero_top + target as f32 * row_pitch;
      let rect = egui::Rect::from_min_max(
        egui::pos2(ui.min_rect().left(), top),
        egui::pos2(ui.min_rect().right(), top + row_height),
      );
      ui.scroll_to_rect(rect, Some(Align::Center));
    }
    for index in range {
      let line = &state.program[index];
      let is_current = index == current && view.connection == ConnectionState::Streaming;
      // Executed lines dim out; the current line is emphasised with the accent and an inset highlight; pending
      // lines sit at the default text colour.
      let color = if is_current {
        palette.accent
      } else if index < current {
        palette.text_disabled
      } else {
        palette.text
      };
      let row = RichText::new(format!("{:>5}  {line}", index + 1)).monospace().color(color);
      if is_current {
        // Highlight the executing line with the accent-tinted inset the design uses (bg + left accent border).
        egui::Frame::new().fill(palette.accent.gamma_multiply(0.12)).inner_margin(egui::Margin {
          left: 4,
          right: 0,
          top: 0,
          bottom: 0,
        }).show(ui, |ui| {
          ui.label(row);
        });
      } else {
        ui.label(row);
      }
    }
  });
  // Record the row we just followed so the next frame only re-scrolls when the executing line advances again,
  // leaving an operator who has scrolled back to read an earlier line undisturbed until the cursor moves.
  if let Some(target) = follow {
    state.program_followed_line = Some(target);
  }
}

/// Render the Console tab body: a rolling, colour-tagged log (chevron coloured by line type) above an
/// auto-scroll toggle and the manual-command entry line with a Send button (design §03). The dock's shared tab
/// strip (see [`dock`]) carries the tab labels and the progress readout; this draws only the tab's content.
pub(crate) fn console_body(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  // The auto-scroll toggle sits above the log within the Console body (the mock's strip is now shared by both
  // tabs, so the toggle moves into the body where it only applies to the console).
  ui.horizontal(|ui| {
    ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
      ui.checkbox(&mut state.auto_scroll, crate::tr!("console-auto-scroll"));
      ui.checkbox(&mut state.verbose, crate::tr!("console-verbose"));
    });
  });

  // Map the visible rows back onto the full buffer: in non-verbose mode bare `ok` acks are dropped so the row
  // virtualisation below counts and indexes only the lines actually drawn.
  let visible: Vec<usize> = view
    .console
    .iter()
    .enumerate()
    .filter(|(_, entry)| state.verbose || !is_ok_noise(entry))
    .map(|(index, _)| index)
    .collect();

  let row_height = ui.text_style_height(&egui::TextStyle::Monospace);
  // The manual-command (MDI) strip is ANCHORED AS AN INNER BOTTOM PANEL, and the log then fills the remainder
  // exactly. This shape is load-bearing twice over:
  // - A resizable egui panel persists its CONTENT's measured rect as next frame's panel size, so the previous
  //   `available − reserved` arithmetic — off by a sub-pixel of font-metric rounding — fed its error back 1:1
  //   and the dock crept larger a pixel every few frames with no input at all (the user-reported self-resizing
  //   console). With the strip panel-pinned and the log filling to it, the content always measures EXACTLY the
  //   panel height and the stored size is a fixed point.
  // - The strip can never be pushed below the dock floor by a greedy fill-height scroll area (the older
  //   vanished-MDI bug the reservation arithmetic was originally added for) — the panel owns its space.
  // Height budget: the strip's real height depends on the font stack (the mono row is ~15.1px under the test
  // fonts but taller under the app's JetBrains Mono + themed spacing), so the slot carries generous headroom —
  // 4px top breathing room between the last log row and the strip plus slack over the tallest observed content.
  // Content that still outgrew the slot would clip at the pane floor, NOT resize anything: the dock's height is
  // a share of the egui_tiles split, which pane content structurally cannot alter.
  egui::Panel::bottom("dock-mdi")
    .exact_size(Metrics::MDI_ROW_H + 8.0)
    .resizable(false)
    .show_separator_line(false)
    .frame(egui::Frame::new().inner_margin(egui::Margin { left: 0, right: 0, top: 4, bottom: 0 }))
    .show(ui, |ui| mdi_strip(ui, view, state, sink));
  // `min_scrolled_height(0)`: egui's 64px default floor would force the log BELOW the reserved space at the
  // split's minimum pane height and paint it over the MDI strip; the log may shrink to a sliver instead — it
  // still scrolls, and the MDI line always stays whole.
  let scroll = ScrollArea::vertical().auto_shrink([false, false]).min_scrolled_height(0.0)
    .stick_to_bottom(state.auto_scroll).show_rows(
    ui,
    row_height,
    visible.len(),
    |ui, range| {
      for row in range {
        if let Some(entry) = visible.get(row).and_then(|&index| view.console.get(index)) {
          let (chevron, color) = console_line_style(palette, entry.source, &entry.text);
          ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 4.0;
            ui.label(RichText::new(chevron).monospace().color(color));
            ui.label(RichText::new(&entry.text).monospace().color(palette.text));
          });
        }
      }
    },
  );
  // Right-clicking anywhere over the console viewport (a row or the empty space below the last line) offers a
  // single "Clear" action. `interact` over the scroll-area rect makes the whole body — not just a painted row —
  // catch the secondary click, so the menu is reliable on a sparse or empty console.
  let console_rect = scroll.inner_rect;
  ui.interact(console_rect, ui.id().with("console_context"), egui::Sense::click()).context_menu(|ui| {
    if ui.button(crate::tr!("btn-clear")).clicked() {
      sink.push(Intent::ClearConsole);
      ui.close();
    }
  });

}

/// The manual-command entry (MDI) strip: a recessed command line — inset fill with a hairline recess border, a
/// blue `›` prompt echoing the console's sent-line chevron, a frameless MONOSPACE field (commands are code, and
/// the field should read like the log it feeds), and the Send button as the row's one filled-accent action.
/// Sends on Enter or the button, only while connected; ↑/↓ recall previously sent lines
/// ([`crate::app::mdi::MdiHistory`]). Hosted by [`console_body`]'s inner bottom panel, which pins it to the dock floor.
fn mdi_strip(ui: &mut egui::Ui, view: &ViewState, state: &mut UiState, sink: &mut IntentSink) {
  let palette = state.style.palette;
  let connected = view.connection.is_connected();
  egui::Frame::new()
    .fill(palette.inset)
    .stroke(egui::Stroke::new(1.0, palette.border_recess))
    .corner_radius(Metrics::CONTROL_RADIUS)
    .inner_margin(egui::Margin::symmetric(8, 4))
    .show(ui, |ui| {
      ui.horizontal(|ui| {
        let prompt_color = if connected { palette.log_sent } else { palette.text_disabled };
        ui.label(RichText::new("›").monospace().size(13.0).color(prompt_color));
        let send_w = Metrics::SEND_PAD_X * 2.0 + 32.0;
        // Give the field an EXPLICIT width — the row's available width minus the Send button and one inter-widget
        // gap — floored at a usable minimum. (A trailing widget added after a full-width field would otherwise be
        // pushed out, and a field added after a right-to-left child collapses to a sliver — both prior bugs.)
        // `MDI_FIELD_MIN_W` guards a narrow dock so the field never vanishes.
        let gap = ui.spacing().item_spacing.x;
        let field_w = (ui.available_width() - send_w - gap).max(MDI_FIELD_MIN_W);
        let hint = if connected { "$$, G0 X0, …".to_string() } else { crate::tr!("mdi-hint-disconnected") };
        // A TRANSPARENT frame (the strip's inset frame is the visible chrome) whose vertical margin is what
        // actually sizes the field: TextEdit height = row height + frame margins (its `min_size.y` is ignored).
        // The margin is computed from the mono row height so the field lands on EXACTLY the 22px control height
        // of the Send button beside it — inputs and buttons share one height (the user-flagged mismatch).
        let mono_row = ui.text_style_height(&egui::TextStyle::Monospace);
        let field_frame = egui::Frame::new()
          .inner_margin(Metrics::text_field_margin(mono_row, 0))
          .fill(Color32::TRANSPARENT);
        let response = ui.add_enabled(connected, egui::TextEdit::singleline(&mut state.console_input)
          .frame(field_frame)
          .font(egui::TextStyle::Monospace)
          .hint_text(hint)
          .desired_width(field_w));
        // ↑/↓ recall while the field holds focus: swap the buffer for the neighbouring history entry. The
        // navigation policy (dedupe, clamping, the draft stash) is the pure `MdiHistory`; a single-line TextEdit
        // has no use of its own for vertical arrows, so borrowing them is safe.
        if response.has_focus() {
          let (up, down) =
            ui.input(|i| (i.key_pressed(egui::Key::ArrowUp), i.key_pressed(egui::Key::ArrowDown)));
          if up {
            if let Some(previous) = state.mdi_history.up(&state.console_input) {
              state.console_input = previous;
            }
          } else if down && let Some(next) = state.mdi_history.down() {
            state.console_input = next;
          }
        }
        let send_clicked = ui.add_enabled_ui(connected, |ui| {
          // The one filled-accent control on the strip: Send is the row's action, everything else is entry.
          let send = egui::Button::new(RichText::new(crate::tr!("btn-send")).size(11.5).color(palette.text))
            .fill(palette.accent);
          ui.add_sized(Vec2::new(send_w, Metrics::PANEL_CONTROL_H), send).clicked()
        }).inner;
        // Submit on Enter (the field loses focus carrying the Enter press) or the Send button. The decision —
        // gated on the link being up and the field holding a non-blank line — is the pure [`should_submit_mdi`]
        // so it is unit-tested without a window; only the take/record/push/refocus side effects stay here.
        let enter = response.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
        if should_submit_mdi(connected, enter || send_clicked, &state.console_input) {
          // Clear the field as we take the line so the next command starts fresh, record it for ↑ recall, and
          // re-focus for the operator to keep typing. The line is a normal G-code/`$` command: it goes through the
          // engine's LINE-BUFFERED send path (`Intent::SendLine` → `send_line` → `Command::SendLine`), counted
          // against the RX buffer like any streamed line — NOT a real-time single byte (those are
          // `Command::Realtime`, reserved for `?`/`!`/`~`/`0x18`/jog-cancel).
          let line = std::mem::take(&mut state.console_input);
          state.mdi_history.push(&line);
          sink.push(Intent::SendLine(line));
          response.request_focus();
        }
      });
    });
}

/// Whether a manual-command (MDI) line typed in the console should be submitted this frame: only when the link is
/// connected, a submit gesture fired (Enter or the Send button), and the field holds a non-blank line. A blank or
/// whitespace-only field never submits (so a stray Enter is a no-op), and nothing is sent while disconnected. Pure so
/// the gate is unit-tested without a window; the caller owns the take/push/refocus side effects.
pub(crate) fn should_submit_mdi(connected: bool, submit_gesture: bool, field: &str) -> bool {
  connected && submit_gesture && !field.trim().is_empty()
}

/// Whether a console line is a bare `ok` acknowledgement — the per-line ack the firmware emits for every consumed
/// command. These are hidden when `verbose` is off so continuous jogging does not flood the log. Pure so it is
/// unit-tested. Only `Received` lines count: a literal `ok` the operator typed and we echoed stays visible.
pub(crate) fn is_ok_noise(entry: &LogLine) -> bool {
  entry.source == LogSource::Received && entry.text.trim() == "ok"
}

/// Decide the chevron glyph and colour for one console line, distinguishing status (`<…>`) and info (`[…]`)
/// firmware lines from a plain response, per the design's console line types. Pure so it is unit-tested.
pub(crate) fn console_line_style(palette: Palette, source: LogSource, text: &str) -> (&'static str, Color32) {
  match source {
    LogSource::Sent => ("›", palette.log_sent),
    LogSource::Notice => ("·", palette.log_notice),
    LogSource::Received => {
      if text.starts_with('<') {
        ("‹", palette.log_status)
      } else if text.starts_with('[') {
        ("‹", palette.log_info)
      } else if text.starts_with("error") || text.starts_with("ALARM") {
        ("‹", palette.state_alarm)
      } else {
        ("‹", palette.log_recv)
      }
    }
  }
}
