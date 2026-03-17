//! Simplified orbital mechanics library for games
//! 
//! Used for simulating celestial bodies' orbits using basic Keplerian mechanics
//! and return information on the relative positions and directions between
//! those bodies from various points of reference for rendering in game engines.
//! 
//! This library is inteded for use either with [the Bevy engine](https://bevyengine.org)
//! or with an as-yet-unbuilt wrapper library for [the Godot engine](https://godotengine.org/)
//! for a personal project of mine.
//! 
//! All of the math and terminology is based on [*Orbital Mechanics*](http://www.braeunig.us/space/orbmech.htm),
//! a web article by Robert A. Braeunig


pub mod constants;
mod body; pub use body::*;
mod database; pub use database::*;
mod elements; pub use elements::*;
#[cfg(any(feature="prefab", test))]
pub mod prefab;
#[cfg(test)]
mod problems;

#[cfg(feature="bevy")]
mod feat_bevy;
#[cfg(feature="bevy")]
pub use feat_bevy::*;
#[cfg(feature="godot")]
pub mod feat_godot;
