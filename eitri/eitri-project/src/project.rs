//! [`Project`] — the top-level document: a name plus its [`ObjectCollection`].
//!
//! A freshly loaded project is *un-hydrated*: Gerber/Excellon objects carry their source text but not their parsed
//! geometry (deserialization is pure and must not run the parsers). [`Project::hydrate`] re-derives that geometry,
//! threading the engine's [`ProgressReporter`]/[`CancelToken`] because re-parsing a large board is exactly the kind
//! of long operation the caller wants to keep off the UI thread and be able to cancel.

use crate::collection::ObjectCollection;
use crate::error::Result;
use crate::object::ObjectPayload;
use eitri_core::{CancelToken, ProgressReporter};
use eitri_excellon::parse_excellon;
use eitri_gerber::parse_gerber;

/// The in-memory project document.
#[derive(Debug, Clone)]
pub struct Project {
  /// Project name.
  pub name: String,
  /// The objects and groups.
  pub collection: ObjectCollection,
}

impl Project {
  /// A new, empty project.
  pub fn new(name: impl Into<String>) -> Project {
    Project { name: name.into(), collection: ObjectCollection::new() }
  }

  /// Re-derive the parsed geometry for every Gerber/Excellon object from its embedded source, filling the parse
  /// caches that deserialization left empty. Objects whose cache is already populated are re-parsed too, so this is
  /// idempotent. Excellon is re-parsed with format inference (`None` override) — faithful for any file that declares
  /// or unambiguously implies its number format, which covers real-world drill files.
  pub fn hydrate(&mut self, progress: &ProgressReporter, cancel: &CancelToken) -> Result<()> {
    for id in self.collection.iter().map(|o| o.meta.id).collect::<Vec<_>>() {
      cancel.check()?;
      let Some(object) = self.collection.get_mut(id) else { continue };
      match &mut object.payload {
        ObjectPayload::Gerber(gerber) => {
          let image = parse_gerber(&gerber.source, progress, cancel).map_err(eitri_core::Error::from)?;
          gerber.image = Some(image);
        }
        ObjectPayload::Excellon(excellon) => {
          let image =
            parse_excellon(&excellon.source, None, progress, cancel).map_err(eitri_core::Error::from)?;
          excellon.image = Some(image);
        }
        ObjectPayload::Geometry(_) | ObjectPayload::CncJob(_) => {}
      }
    }
    Ok(())
  }
}
