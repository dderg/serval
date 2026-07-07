// Every FFI entry projects to &mut FgState or &mut IsrState (disjoint memory) via
// core::ptr::addr_of! + UnsafeCell::raw_get; no &mut RuntimeContext is ever materialised.
// See docs/rewrite/mcu-c-rust-boundary.md.

#![allow(unsafe_code)]

#[cfg(feature = "header-runtime")]
pub mod exports;
