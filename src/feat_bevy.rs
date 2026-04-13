use std::{collections::hash_map::Iter, fmt::{Debug, Display}, hash::Hash};
use bevy::prelude::*;
use nalgebra::Vector3;
use num_traits::FromPrimitive;
use crate::{Database, DatabaseEntry};


#[derive(Default, Resource)]
pub struct BevyPlanetDatabase<H> {
    database: Database<H, f32>
}
impl<H> BevyPlanetDatabase<H> where H: Clone + Debug + Display + Eq + Hash + FromPrimitive + Ord {
    pub fn get_entry(&self, handle: &H) -> &DatabaseEntry<H, f32> {
        self.database.get_entry(handle)
    }
    pub fn get_satellites(&self, handle: &H) -> Vec<H> {
        self.database.get_satellites(handle)
    }
	pub fn get_parents(&self, handle: &H, inclusive: bool) -> Vec<H> {
		self.database.get_parents(handle, inclusive)
	}
	pub fn position_at_mean_anomaly(&self, handle: &H, mean_anomaly: f32) -> Vec3 {
		vec_nalgebra_to_bevy(self.database.position_at_mean_anomaly(handle, mean_anomaly))
	}
    pub fn position_at_time(&self, handle: &H, time: f32) -> Vec3 {
        vec_nalgebra_to_bevy(self.database.position_at_time(handle, time))
    }
    pub fn absolute_position_at_time(&self, handle: &H, time: f32) -> Vec3 {
        vec_nalgebra_to_bevy(self.database.absolute_position_at_time(handle, time))
    }
	pub fn relative_position(&self, origin: &H, relative: &H, time: f32) -> Option<Vec3> {
		match self.database.relative_position(origin, relative, time) {
			Ok(vector) => Some(vec_nalgebra_to_bevy(vector)),
			Err(_) => None,
		}
	}
    pub fn radius_soi(&self, handle: &H) -> f32 {
        self.database.radius_soi(handle)
    }
    #[cfg(feature="prefab")]
    pub fn with_solar_system(mut self) -> Self {
        self.database = self.database.with_solar_system();
        self
    }
	pub fn mean_anomaly_at_time(&self, handle: &H, time: f32) -> f32 {
		self.database.mean_anomaly_at_time(handle, time)
	}
    pub fn iter(&self) -> Iter<'_, H, DatabaseEntry<H, f32>> {
        self.database.iter()
    }
}

pub fn vec_nalgebra_to_bevy(input: Vector3<f32>) -> Vec3 {
    Vec3::new(input.x, input.y, input.z)
}
