use std::{
	collections::{hash_map::Iter, HashMap},
	fmt::{Debug, Display},
	hash::Hash, ops::SubAssign
};
use nalgebra::{RealField, Rotation3, SimdRealField, SimdValue, Vector3};
use num_traits::{Float, FromPrimitive};
use crate::{constants::f64::CONVERT_DEG_TO_RAD, Body, OrbitalElements};

#[cfg(feature="bevy")]
use bevy::prelude::*;

/// Holds the data for all the bodies being simulated
/// 
/// This is the main source of information for game engine implementations. The game engine should
/// feed its celestial body information into this database, and then query it to get the results of
/// calculations back.
/// 
/// `T` is the type used for the floating point data stored inside the database, and `H` is the
/// hashable type used for handles to celestial bodies which are used to retrieve a specific body
/// from the database and also define parent/child relationships
#[cfg_attr(feature="bevy", derive(Resource))]
pub struct Database<H, T> {
	bodies: HashMap<H, DatabaseEntry<H, T>>,
}
impl<H, T> Database<H, T> where H: Clone + Eq + Hash + FromPrimitive, T: Clone + Float + FromPrimitive + SubAssign {
	/// Adds a new entry to the database
	pub fn add_entry(&mut self, handle: H, entry: DatabaseEntry<H, T>) {
		self.bodies.insert(handle, entry);
	}
	/// Gets the entry from the database with the given handle
	pub fn get_entry(&self, handle: &H) -> &DatabaseEntry<H, T> where H: Debug {
		let error_msg = format!("No body in database with ID {:?}", handle);
		self.bodies.get(handle).expect(&error_msg)
	}
	/// Gets the position of the given body at the given time since epoch in seconds
	pub fn position_at_mean_anomaly(&self, handle: &H, mean_anomaly: T) -> Vector3<T> where H: Debug, T: RealField + SimdValue + SimdRealField {
		let zero = T::from_f32(0.0).unwrap();
		let one = T::from_f32(1.0).unwrap();
		let two = T::from_f32(2.0).unwrap();
		let x_axis = Vector3::new(one, zero, zero);
		let y_axis = Vector3::new(zero, one, zero);
		let orbiting_body = self.bodies.get(&handle).unwrap();
		if let Some(orbit) = &orbiting_body.orbit {
			let parent = self.get_entry(&orbiting_body.parent.clone().unwrap());
			let parent_axis_rot: Rotation3<T> = Rotation3::new(x_axis * parent.info.axial_tilt_rad());
			let parent_up: Vector3<T> = parent_axis_rot * y_axis;
			let true_anomaly = mean_anomaly + two * orbit.eccentricity * Float::sin(mean_anomaly) + T::from_f64(1.25).unwrap() * Float::powi(orbit.eccentricity, 2) * Float::sin(two * mean_anomaly);
			let radius = orbit.semimajor_axis * (one - Float::powi(orbit.eccentricity, 2)) / (one + orbit.eccentricity * Float::cos(true_anomaly));
			let rot_true_anomaly = Rotation3::new(parent_up * true_anomaly);
			let rot_long_of_ascending_node = Rotation3::new(parent_up * orbit.long_of_ascending_node);
			let dir_ascending_node = rot_long_of_ascending_node * x_axis;
			let dir_normal = x_axis.cross(&dir_ascending_node);
			let rot_inclination = Rotation3::new(dir_ascending_node * orbit.inclination);
			let rot_arg_of_periapsis = Rotation3::new(dir_normal * orbit.arg_of_periapsis);
			let direction = rot_inclination * rot_arg_of_periapsis * rot_true_anomaly * x_axis;
			return direction * radius;
		} else {
			return Vector3::new(zero, zero, zero);
		}
	}
	pub fn position_at_time(&self, handle: &H, time: T) -> Vector3<T> where H: Debug, T: RealField {
		let orbiting_body = self.bodies.get(handle).unwrap();
		if orbiting_body.orbit.is_some() {
			let mean_anomaly = self.mean_anomaly_at_time(handle, time);
			return self.position_at_mean_anomaly(handle, mean_anomaly);
		} else {
			let zero = T::from_f32(0.0).unwrap();
			return Vector3::new(zero, zero, zero);
		}
	}
	/// Get the position of one body relative to another
	pub fn relative_position(&self, origin: &H, relative: &H, time: T) -> Result<Vector3<T>, String> where H: Debug + Display + Ord, T: RealField + SimdValue + SimdRealField {
		// println!("Finding relative position between origin body {} and relative body {}", origin, relative);
		let mut relative_heirarchy: Vec<H> = self.get_parents(relative, true);
		relative_heirarchy.reverse();
		// println!("Relative heirarchy: {:?}", relative_heirarchy);
		let zero = T::from_f32(0.0).unwrap();
		let mut relative_position = Vector3::new(zero, zero, zero);
		let mut entry = self.get_entry(origin);
		// println!("\tSubtracting orbital position of {}", origin);
		relative_position -= self.position_at_time(origin, time);
		// if origin body is already in the parent heirarchy of the relative body, find the relative body position
		if let Ok(parent_relative_index) = relative_heirarchy.binary_search(origin) {
			// println!("Reached heirarchy intersection at relative index {}", parent_relative_index);
			let mut index = parent_relative_index;
			let mut handle;
			while index < relative_heirarchy.len() {
				handle = &relative_heirarchy[index];
				entry = self.get_entry(handle);
				// println!("\tAdding orbital position of {}", handle);
				relative_position += self.position_at_time(handle, time);
				// println!("Checking if body at index {} ({}) is the relative body {}", index, handle, relative);
				if handle == relative {
					return Ok(relative_position);
				}
				// println!("Body at index {} ({}) is not the relative body {}. Incrementing index and trying again", index, handle, relative);
				index += 1;
			}
		}
		while let Some(parent_handle) = &entry.parent {
			entry = self.get_entry(parent_handle);
			// println!("\tSubtracting orbital position of {}", parent_handle);
			relative_position -= self.position_at_time(parent_handle, time);
			// if the heirarchy of the relative body contains this body, start summing the orbits in that heirarchy
			if let Ok(parent_relative_index) = relative_heirarchy.binary_search(&parent_handle) {
				// println!("Reached heirarchy intersection at body {}", parent_handle);
				let mut index = parent_relative_index;
				let mut handle;
				while index < relative_heirarchy.len() {
					handle = &relative_heirarchy[index];
					entry = self.get_entry(handle);
					// println!("\tAdding orbital position of {}", handle);
					relative_position += self.position_at_time(handle, time);
					// println!("Checking if body at index {} ({}) is the relative body {}", index, handle, relative);
					if handle == relative {
						return Ok(relative_position);
					}
					// println!("Body at index {} ({}) is not the relative body {}. Incrementing index and trying again", index, handle, relative);
					index += 1;
					
				}
			}
		}
		let err_msg = format!("Failed to find relative position between {} and {}", origin, relative);
		return Err(err_msg);
	}
	pub fn absolute_position_at_time(&self, handle: &H, time: T) -> Vector3<T> where H: Debug, T: RealField + SimdValue + SimdRealField {
		let zero = T::from_f32(0.0).unwrap();
		if let Some(entry) = self.bodies.get(&handle) {
			let parent_position = match &entry.parent {
				Some(parent_handle) => self.absolute_position_at_time(parent_handle, time),
				None => Vector3::new(zero, zero, zero),
			};
			return self.position_at_time(handle, time) + parent_position;
		} else {
			return Vector3::new(zero, zero, zero);
		}
	}
	/// Get a list of handles for satellites of the body with the input handle.
	pub fn get_satellites(&self, body: &H) -> Vec<H> where H: Ord {
		let mut satellites: Vec<H> = Vec::new();
		for (handle, entry) in self.iter() {
			if let Some(parent_handle) = &entry.parent {
				if *parent_handle == *body {
					satellites.push(handle.clone());
				}
			}
		}
		satellites.sort();
		satellites
	}
	/// Get the heirarchy of parent bodies of the input body
	/// 
	/// Bodies are returned in order, starting from the lowest body in the heirarchy to the highest.
	/// For example, calling this function on the moon with `inclusive` set to `true` would return
	/// `[moon, earth, sun]`
	pub fn get_parents(&self, body: &H, inclusive: bool) -> Vec<H> where H: Debug {
		let mut handles: Vec<H> = match inclusive {
			true => vec![body.clone()],
			false => Vec::new(),
		};
		let mut body_entry = self.get_entry(body);
		while let Some(parent_handle) = &body_entry.parent {
			handles.push(parent_handle.clone());
			body_entry = self.get_entry(&parent_handle);
		}
		return handles;
	}
	/// Gets the body whose sphere of influence dominates the given `position` in space, relative to the `origin` body
	pub fn get_soi(&self, position: Vector3<T>, origin: &H, time: T) -> Option<H>
	where H: Debug + Display + Ord, T: RealField + SimdValue + SimdRealField
	{
		// If in `origin` SOI, check `origin`'s satellites
		if self.is_in_soi(position, &origin, &origin, time) {
			match self.get_satellite_soi(position, origin, origin, time) {
				Some(body) => return Some(body),
				None => return Some(origin.clone()),
			};
		}
		// Else check `origin`'s parent bodies
		else {
			for parent in self.get_parents(&origin, false).iter() {
				if self.is_in_soi(position, parent, &origin, time) {
					return Some(parent.clone());
				}
			}
		}
		// If no SOIs match for the satellites or parent bodies, return `None`
		None
	}
	/// Returns which satellite of `body` the given `position` lies within the sphere of influence of.
	/// 
	/// Does not check if the satellite lies within the sphere of influence of `body` itself. If
	/// the point is not in the SOI of any of its satellites, it will return `None` even if the
	/// position is in the SOI of `body`
	pub fn get_satellite_soi(&self, position: Vector3<T>, body: &H, origin: &H, time: T) -> Option<H>
	where H: Debug + Display + Ord, T: RealField + SimdValue + SimdRealField
	{
		let satellites = self.get_satellites(&body);
		for satellite in satellites.iter() {
			if self.is_in_soi(position, satellite, &origin, time) {
				match self.get_satellite_soi(position, satellite, origin, time) {
					Some(body) => return Some(body),
					None => return Some(satellite.clone()),
				};
			}
		}
		None
	}
	/// Returns whether or not the given `position` lies within the sphere of influence of `body`, relative to `origin`
	pub fn is_in_soi(&self, position: Vector3<T>, body: &H, origin: &H, time: T) -> bool
	where H: Debug + Display + Ord, T: RealField + SimdValue + SimdRealField {
		let body_position: Vector3<T> = self.relative_position(&origin, &body, time).unwrap();
		let test_distance: T = body_position.metric_distance(&position);
		test_distance <= self.radius_soi(&body)
	}
	/// Gets the combined mass of a body and all its satellites
	pub fn get_combined_mass_kg(&self, body: &H) -> T where H: Debug + Ord {
		let body_entry = self.get_entry(body);
		let mut total_mass = body_entry.info.mass_kg();
		for satellite_handle in self.get_satellites(body) {
			total_mass = total_mass + self.get_combined_mass_kg(&satellite_handle);
		}
		return total_mass;
	}
	/// Calculate the radius of the sphere of influence of the body with the given handle
	pub fn radius_soi(&self, handle: &H) -> T where H: Debug + Ord {
		let orbiting_body = self.bodies.get(&handle).unwrap();
		let orbiting_body_info = orbiting_body.info.clone();
		let orbiting_body_mass = self.get_combined_mass_kg(handle);
		if let Some(orbit) = &orbiting_body.orbit {
			let parent_body = self.bodies.get(&orbiting_body.parent.clone().unwrap()).unwrap();
			let parent_body_info = parent_body.info.clone();
			let exponent = T::from_f64(2.0 / 5.0).unwrap();
			return orbit.semimajor_axis * (orbiting_body_mass / parent_body_info.mass_kg()).powf(exponent);
		} else {
			let minimum_gravity = T::from_f64(0.0000005).unwrap();
			return orbiting_body_info.distance_of_gravity(minimum_gravity);
		}
	}
	pub fn mean_anomaly_at_time(&self, handle: &H, time: T) -> T where H: Debug {
		let orbiting_entry = self.get_entry(handle);
		if let Some(parent_handle) = &orbiting_entry.parent {
			let orbit = orbiting_entry.orbit.clone().unwrap();
			let parent_entry = self.get_entry(parent_handle);
			let n = Float::sqrt(parent_entry.gm() / Float::powi(orbit.semimajor_axis, 3));
			let mean_anomaly = orbiting_entry.mean_anomaly_at_epoch + n * time; 
			return mean_anomaly;
		} else {
			return T::from_f32(0.0).unwrap();
		}
	}
	pub fn iter(&self) -> Iter<'_, H, DatabaseEntry<H, T>> {
		self.bodies.iter()
	}
}
impl<H, T> Default for Database<H, T> {
	fn default() -> Self {
		Self{ bodies: HashMap::new() }
	}
}


pub struct DatabaseEntry<H, T> {
	pub parent: Option<H>,
	pub name: String,
	pub info: Body<T>,
	pub orbit: Option<OrbitalElements<T>>,
	pub mean_anomaly_at_epoch: T,
	pub scale: T,
}
impl<H, T> DatabaseEntry<H, T> where T: Float + FromPrimitive + SubAssign {
	pub fn new<S>(info: Body<T>, name: S) -> Self where S: Into<String> {
		Self{
			info, name: name.into(),
			parent: None, orbit: None, mean_anomaly_at_epoch: T::from_f64(0.0).unwrap(),
			scale: T::from_f64(1.0 / 3_000_000.0).unwrap(),
		}
	}
	pub fn with_parent(mut self, parent_handle: H, orbital_elements: OrbitalElements<T>) -> Self {
		self.parent = Some(parent_handle);
		self.orbit = Some(orbital_elements);
		self
	}
	pub fn with_scale(mut self, scale: T) -> Self {
		self.scale = scale;
		self
	}
	pub fn with_mean_anomaly_deg(mut self, mean_anomaly: T) -> Self {
		self.mean_anomaly_at_epoch = mean_anomaly * T::from_f64(CONVERT_DEG_TO_RAD).unwrap();
		let circle = T::from_f64(360.0).unwrap();
		while self.mean_anomaly_at_epoch > circle {
			self.mean_anomaly_at_epoch -= circle;
		}
		self
	}
	pub fn gm(&self) -> T {
		self.info.gm()
	}
}


#[cfg(test)]
mod tests {
	use super::*;
	use crate::prefab::handles::*;

	#[test]
	fn get_satellites() {
		let database = Database::<u16, f32>::default().with_solar_system();
		let satellites = database.get_satellites(&HANDLE_EARTH);
		assert_eq!(1, satellites.len());
		assert!(satellites.contains(&HANDLE_LUNA));
		let satellites = database.get_satellites(&HANDLE_MARS);
		assert_eq!(2, satellites.len());
		assert!(satellites.contains(&HANDLE_PHOBOS));
		assert!(satellites.contains(&HANDLE_DEIMOS));
	}

	#[test]
	fn get_parents() {
		let database = Database::<u16, f32>::default().with_solar_system();
		let heirarchy = database.get_parents(&HANDLE_SOL, false);
		assert_eq!(0, heirarchy.len());
		let heirarchy = database.get_parents(&HANDLE_SOL, true);
		assert_eq!(1, heirarchy.len());
		assert_eq!(HANDLE_SOL, heirarchy[0]);
		let heirarchy = database.get_parents(&HANDLE_MARS, false);
		assert_eq!(1, heirarchy.len());
		assert_eq!(HANDLE_SOL, heirarchy[0]);
		let heirarchy = database.get_parents(&HANDLE_MARS, true);
		assert_eq!(2, heirarchy.len());
		assert_eq!(HANDLE_MARS, heirarchy[0]);
		assert_eq!(HANDLE_SOL, heirarchy[1]);
		let heirarchy = database.get_parents(&HANDLE_DEIMOS, false);
		assert_eq!(2, heirarchy.len());
		assert_eq!(HANDLE_MARS, heirarchy[0]);
		assert_eq!(HANDLE_SOL, heirarchy[1]);
		let heirarchy = database.get_parents(&HANDLE_DEIMOS, true);
		assert_eq!(3, heirarchy.len());
		assert_eq!(HANDLE_DEIMOS, heirarchy[0]);
		assert_eq!(HANDLE_MARS, heirarchy[1]);
		assert_eq!(HANDLE_SOL, heirarchy[2]);
	}

	#[test]
	fn is_in_soi() {
		const SUN_HANDLE: u8 = 0;
		const PLANET_HANDLE: u8 = 1;
		const MOON_HANDLE: u8 = 2;
		const TIME: f32 = 0.0;

		// setup
		let mut database: Database<u8, f32> = Database::default();
		let sun = DatabaseEntry::new(
			Body::default().with_mass_kg(300000000.0), // SOI ~ 200m
			"Sun",
		);
		let planet = DatabaseEntry::new(
			Body::default().with_mass_kg(6000000.0), // SOI ~ 20m distance 100
			"Planet",
		).with_parent(SUN_HANDLE, OrbitalElements::default().with_semimajor_axis_m(100.0));
		let moon = DatabaseEntry::new(
			Body::default().with_mass_kg(150000.0), // SOI ~ 2m distance 10
			"Moon",
		).with_parent(PLANET_HANDLE, OrbitalElements::default().with_semimajor_axis_m(10.0));
		database.add_entry(SUN_HANDLE, sun);
		database.add_entry(PLANET_HANDLE, planet);
		database.add_entry(MOON_HANDLE, moon);
		println!(
			"SOI Radius\n\tSun: {}\n\tPlanet: {}\n\tMoon: {}",
			database.radius_soi(&SUN_HANDLE),
			database.radius_soi(&PLANET_HANDLE),
			database.radius_soi(&MOON_HANDLE),
		);
		println!(
			"Relative positions:\n\tPlanet: {:?}\n\tMoon: {:?}",
			database.relative_position(&SUN_HANDLE, &PLANET_HANDLE, TIME),
			database.relative_position(&PLANET_HANDLE, &MOON_HANDLE, TIME),
		);
		
		// test
		let test_cases: Vec<(Vector3<f32>, u8, u8, bool)> = vec![
			// Sun relative
			(Vector3::new(0.0, 0.0, 0.0), SUN_HANDLE, SUN_HANDLE, true),
			(Vector3::new(0.0, 0.0, 0.0), PLANET_HANDLE, SUN_HANDLE, false),
			(Vector3::new(0.0, 0.0, 0.0), MOON_HANDLE, SUN_HANDLE, false),
			(Vector3::new(100.0, 0.0, 0.0), SUN_HANDLE, SUN_HANDLE, true),
			(Vector3::new(100.0, 0.0, 0.0), PLANET_HANDLE, SUN_HANDLE, true),
			(Vector3::new(100.0, 0.0, 0.0), MOON_HANDLE, SUN_HANDLE, false),
			(Vector3::new(110.0, 0.0, 0.0), SUN_HANDLE, SUN_HANDLE, true),
			(Vector3::new(110.0, 0.0, 0.0), PLANET_HANDLE, SUN_HANDLE, true),
			(Vector3::new(110.0, 0.0, 0.0), MOON_HANDLE, SUN_HANDLE, true),
			(Vector3::new(300.0, 0.0, 0.0), SUN_HANDLE, SUN_HANDLE, false),
			(Vector3::new(300.0, 0.0, 0.0), PLANET_HANDLE, SUN_HANDLE, false),
			(Vector3::new(300.0, 0.0, 0.0), MOON_HANDLE, SUN_HANDLE, false),
			// Planet relative
			(Vector3::new(0.0, 0.0, 0.0), SUN_HANDLE, PLANET_HANDLE, true),
			(Vector3::new(0.0, 0.0, 0.0), PLANET_HANDLE, PLANET_HANDLE, true),
			(Vector3::new(0.0, 0.0, 0.0), MOON_HANDLE, PLANET_HANDLE, false),
			(Vector3::new(10.0, 0.0, 0.0), SUN_HANDLE, PLANET_HANDLE, true),
			(Vector3::new(10.0, 0.0, 0.0), PLANET_HANDLE, PLANET_HANDLE, true),
			(Vector3::new(10.0, 0.0, 0.0), MOON_HANDLE, PLANET_HANDLE, true),
			(Vector3::new(-30.0, 0.0, 0.0), SUN_HANDLE, PLANET_HANDLE, true),
			(Vector3::new(-30.0, 0.0, 0.0), PLANET_HANDLE, PLANET_HANDLE, false),
			(Vector3::new(-30.0, 0.0, 0.0), MOON_HANDLE, PLANET_HANDLE, false),
			(Vector3::new(200.0, 0.0, 0.0), SUN_HANDLE, PLANET_HANDLE, false),
			(Vector3::new(200.0, 0.0, 0.0), PLANET_HANDLE, PLANET_HANDLE, false),
			(Vector3::new(200.0, 0.0, 0.0), MOON_HANDLE, PLANET_HANDLE, false),
			// Moon relative
			(Vector3::new(0.0, 0.0, 0.0), SUN_HANDLE, MOON_HANDLE, true),
			(Vector3::new(0.0, 0.0, 0.0), PLANET_HANDLE, MOON_HANDLE, true),
			(Vector3::new(0.0, 0.0, 0.0), MOON_HANDLE, MOON_HANDLE, true),
			(Vector3::new(3.0, 0.0, 0.0), SUN_HANDLE, MOON_HANDLE, true),
			(Vector3::new(3.0, 0.0, 0.0), PLANET_HANDLE, MOON_HANDLE, true),
			(Vector3::new(3.0, 0.0, 0.0), MOON_HANDLE, MOON_HANDLE, false),
			(Vector3::new(30.0, 0.0, 0.0), SUN_HANDLE, MOON_HANDLE, true),
			(Vector3::new(30.0, 0.0, 0.0), PLANET_HANDLE, MOON_HANDLE, false),
			(Vector3::new(30.0, 0.0, 0.0), MOON_HANDLE, MOON_HANDLE, false),
			(Vector3::new(200.0, 0.0, 0.0), SUN_HANDLE, MOON_HANDLE, false),
			(Vector3::new(200.0, 0.0, 0.0), PLANET_HANDLE, MOON_HANDLE, false),
			(Vector3::new(200.0, 0.0, 0.0), MOON_HANDLE, MOON_HANDLE, false),
		];
		for (position, body, origin, expected) in test_cases.iter() {
			let result: bool = database.is_in_soi(*position, body, origin, 0.0);
			assert_eq!(
				*expected, result,
				"Expected point database.is_in_soi() to return {} for body {} at position {:?} relative to {}",
				expected, body, position, origin,
			);
		}
	}

	#[test]
	fn get_soi() {
		const SUN_HANDLE: u8 = 0;
		const PLANET_HANDLE: u8 = 1;
		const MOON_HANDLE: u8 = 2;
		const TIME: f32 = 0.0;

		// setup
		let mut database: Database<u8, f32> = Database::default();
		let sun = DatabaseEntry::new(
			Body::default().with_mass_kg(300000000.0), // SOI ~ 200m
			"Sun",
		);
		let planet = DatabaseEntry::new(
			Body::default().with_mass_kg(6000000.0), // SOI ~ 20m distance 100
			"Planet",
		).with_parent(SUN_HANDLE, OrbitalElements::default().with_semimajor_axis_m(100.0));
		let moon = DatabaseEntry::new(
			Body::default().with_mass_kg(150000.0), // SOI ~ 2m distance 10
			"Moon",
		).with_parent(PLANET_HANDLE, OrbitalElements::default().with_semimajor_axis_m(10.0));
		database.add_entry(SUN_HANDLE, sun);
		database.add_entry(PLANET_HANDLE, planet);
		database.add_entry(MOON_HANDLE, moon);
		println!(
			"SOI Radius\n\tSun: {}\n\tPlanet: {}\n\tMoon: {}",
			database.radius_soi(&SUN_HANDLE),
			database.radius_soi(&PLANET_HANDLE),
			database.radius_soi(&MOON_HANDLE),
		);
		println!(
			"Relative positions:\n\tPlanet: {:?}\n\tMoon: {:?}",
			database.relative_position(&SUN_HANDLE, &PLANET_HANDLE, TIME),
			database.relative_position(&PLANET_HANDLE, &MOON_HANDLE, TIME),
		);
		
		// test
		let test_cases: Vec<(Vector3<f32>, u8, Option<u8>)> = vec![
			// Sun relative
			(Vector3::new(0.0, 0.0, 0.0), SUN_HANDLE, Some(SUN_HANDLE)),
			(Vector3::new(100.0, 0.0, 0.0), SUN_HANDLE, Some(PLANET_HANDLE)),
			(Vector3::new(110.0, 0.0, 0.0), SUN_HANDLE, Some(MOON_HANDLE)),
			(Vector3::new(300.0, 0.0, 0.0), SUN_HANDLE, None),
			// planet relative
			(Vector3::new(0.0, 0.0, 0.0), PLANET_HANDLE, Some(PLANET_HANDLE)),
			(Vector3::new(10.0, 0.0, 0.0), PLANET_HANDLE, Some(MOON_HANDLE)),
			(Vector3::new(30.0, 0.0, 0.0), PLANET_HANDLE, Some(SUN_HANDLE)),
			(Vector3::new(200.0, 0.0, 0.0), PLANET_HANDLE, None),
			// Moon relative
			(Vector3::new(0.0, 0.0, 0.0), MOON_HANDLE, Some(MOON_HANDLE)),
			(Vector3::new(3.0, 0.0, 0.0), MOON_HANDLE, Some(PLANET_HANDLE)),
			(Vector3::new(30.0, 0.0, 0.0), MOON_HANDLE, Some(SUN_HANDLE)),
			(Vector3::new(200.0, 0.0, 0.0), MOON_HANDLE, None),
		];
		for (position, origin, expected) in test_cases.iter() {
			let result: Option<u8> = database.get_soi(*position, origin, TIME);
			assert_eq!(
				*expected, result,
				"Expected point database.get_soi() to return {:?} at position {:?} relative to {}",
				expected, position, origin,
			);
		}
	}
}