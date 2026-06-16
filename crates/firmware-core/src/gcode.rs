//! GCode tokenizer and parser (DOC-04).
//!
//! A streaming, allocation-free parser: each CR/LF-terminated line is lexed into `(letter, f32)`
//! word pairs (whitespace and `(...)`/`;` comments stripped, case-insensitive), validated against
//! the supported modal groups, applied to modal state, and emitted as a planner command.
//!
//! Status: not yet implemented.
// TODO(DOC-04): implement the streaming lexer, modal-group validation, and PlannerCommand emission.
