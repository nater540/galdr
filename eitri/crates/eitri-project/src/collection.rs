//! [`ObjectCollection`] — the ordered, named set of objects behind a (future) project tree, with no UI dependency.
//!
//! FlatCAM's `ObjectCollection` was a Qt tree model. This is the pure data: insertion-ordered objects keyed by a
//! stable [`ObjectId`], addressable by their unique name, and optionally gathered into named groups. Ids are
//! allocated monotonically and never reused, so a CNC job's back-reference to its parent Gerber survives renames,
//! removals, and a save/load round-trip.

use crate::error::{ProjectError, Result};
use crate::id::ObjectId;
use crate::object::{Object, ObjectMeta, ObjectPayload};
use serde::{Deserialize, Serialize};

/// A named grouping of objects (the data behind a project-tree folder). Membership is by id, so a grouped object
/// can still be renamed or reordered freely.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Group {
  /// Unique group name.
  pub name: String,
  /// Member object ids, in the order they were added.
  pub members: Vec<ObjectId>,
}

/// The ordered, named collection of project objects.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ObjectCollection {
  /// Objects in insertion (display) order.
  objects: Vec<Object>,
  /// Named groups over the objects.
  groups: Vec<Group>,
  /// Monotonic id allocator; the next id handed out. Never decreases, so ids are never reused within a collection.
  next_id: u64,
}

impl Default for ObjectCollection {
  fn default() -> ObjectCollection {
    ObjectCollection::new()
  }
}

impl ObjectCollection {
  /// An empty collection.
  pub fn new() -> ObjectCollection {
    ObjectCollection { objects: Vec::new(), groups: Vec::new(), next_id: 1 }
  }

  /// Add an object, allocating and returning its stable id. The object's [`ObjectMeta::name`] must be unique; the
  /// placeholder id in the supplied metadata is overwritten with the freshly allocated one.
  pub fn add(&mut self, mut meta: ObjectMeta, payload: ObjectPayload) -> Result<ObjectId> {
    if self.by_name(&meta.name).is_some() {
      return Err(ProjectError::DuplicateName(meta.name));
    }
    let id = ObjectId(self.next_id);
    self.next_id += 1;
    meta.id = id;
    self.objects.push(Object { meta, payload });
    Ok(id)
  }

  /// The number of objects.
  pub fn len(&self) -> usize {
    self.objects.len()
  }

  /// Whether the collection holds no objects.
  pub fn is_empty(&self) -> bool {
    self.objects.is_empty()
  }

  /// Borrow an object by id.
  pub fn get(&self, id: ObjectId) -> Option<&Object> {
    self.objects.iter().find(|o| o.meta.id == id)
  }

  /// Mutably borrow an object by id.
  pub fn get_mut(&mut self, id: ObjectId) -> Option<&mut Object> {
    self.objects.iter_mut().find(|o| o.meta.id == id)
  }

  /// Borrow an object by its unique name.
  pub fn by_name(&self, name: &str) -> Option<&Object> {
    self.objects.iter().find(|o| o.meta.name == name)
  }

  /// Iterate objects in insertion order.
  pub fn iter(&self) -> impl Iterator<Item = &Object> {
    self.objects.iter()
  }

  /// Remove an object by id, also dropping it from any group that referenced it. Returns the removed object.
  pub fn remove(&mut self, id: ObjectId) -> Option<Object> {
    let index = self.objects.iter().position(|o| o.meta.id == id)?;
    for group in &mut self.groups {
      group.members.retain(|m| *m != id);
    }
    Some(self.objects.remove(index))
  }

  /// Rename an object, enforcing name uniqueness. The id is unchanged.
  pub fn rename(&mut self, id: ObjectId, new_name: impl Into<String>) -> Result<()> {
    let new_name = new_name.into();
    if let Some(existing) = self.by_name(&new_name) {
      if existing.meta.id != id {
        return Err(ProjectError::DuplicateName(new_name));
      }
    }
    let object = self.get_mut(id).ok_or(ProjectError::UnknownId(id))?;
    object.meta.name = new_name;
    Ok(())
  }

  /// Move an object to a new position in the display order, clamped to the valid range.
  pub fn reorder(&mut self, id: ObjectId, new_index: usize) -> Result<()> {
    let from = self.objects.iter().position(|o| o.meta.id == id).ok_or(ProjectError::UnknownId(id))?;
    let object = self.objects.remove(from);
    let to = new_index.min(self.objects.len());
    self.objects.insert(to, object);
    Ok(())
  }

  /// Create an empty named group.
  pub fn create_group(&mut self, name: impl Into<String>) -> Result<()> {
    let name = name.into();
    if self.groups.iter().any(|g| g.name == name) {
      return Err(ProjectError::DuplicateGroup(name));
    }
    self.groups.push(Group { name, members: Vec::new() });
    Ok(())
  }

  /// Add an object to a group. The group must exist and the object id must be valid; re-adding a member is a no-op.
  pub fn add_to_group(&mut self, group: &str, id: ObjectId) -> Result<()> {
    if self.get(id).is_none() {
      return Err(ProjectError::UnknownId(id));
    }
    let group = self.groups.iter_mut().find(|g| g.name == group).ok_or_else(|| {
      ProjectError::UnknownGroup(group.to_string())
    })?;
    if !group.members.contains(&id) {
      group.members.push(id);
    }
    Ok(())
  }

  /// The member ids of a named group, in add order.
  pub fn group_members(&self, group: &str) -> Option<&[ObjectId]> {
    self.groups.iter().find(|g| g.name == group).map(|g| g.members.as_slice())
  }

  /// Iterate the groups in creation order.
  pub fn groups(&self) -> impl Iterator<Item = &Group> {
    self.groups.iter()
  }
}
