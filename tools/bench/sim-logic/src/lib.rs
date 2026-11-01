//! Pure logic of the F407 printer simulator (see
//! `tools/bench/capture-fw/PHASE2.md`), shared by the firmware and its host
//! tests. No hardware access, no allocation.

#![no_std]

pub mod aux;
pub mod config;
pub mod dir_merge;
pub mod extend;
pub mod thermal;
pub mod tmc;
pub mod tracker;
pub mod walls;

pub const CHANNELS: usize = 4;
