//! Puts a camera in low earth orbit at about the same altitude as the
//! international space station

#[cfg(feature="example")]
mod low_earth_orbit {
	use std::f32::consts::PI;
	pub use bevy::{prelude::*, pbr::wireframe::{Wireframe, WireframePlugin}};
	use game_orbits::{constants::f32::*, Body};

	const SCALE: f32 = 1.0 / 3_000_000.0;
	const ALTITUDE_KM: f32 = 400.0;
	const ECCENTRICITY: f32 = 0.01;
	const PLANET_START_ROTATION: f32 = -1.71;


	#[derive(Resource)]
	pub struct TextElements {
		pub time: Entity,
		pub semimajor_axis: Entity,
		pub eccentricity: Entity,
		pub mean_motion: Entity,
		pub mean_anomaly: Entity,
		pub true_anomaly: Entity,
	}

	#[derive(Component)]
	pub struct Planet;

	#[derive(Component)]
	pub struct Orbit {
		pub time: f32,
		pub semimajor_axis: f32,
		pub eccentricity: f32,
	}
	impl Orbit {
		pub fn with_semimajor_axis(mut self, semimajor_axis: f32) -> Self {
			self.semimajor_axis = semimajor_axis;
			self
		}
		pub fn with_eccentricity(mut self, eccentricity: f32) -> Self {
			self.eccentricity = eccentricity;
			self
		}
		pub fn mean_motion(&self, body: &Body<f32>) -> f32 {
			(body.gm() / self.semimajor_axis.powi(3)).sqrt()
		}
		pub fn mean_anomaly(&self, body: &Body<f32>) -> f32 {
			self.time * self.mean_motion(body)
		}
		pub fn true_anomaly(&self, body: &Body<f32>) -> f32 {
			let mean_anomaly = self.mean_anomaly(body);
			mean_anomaly + 2.0 * self.eccentricity * mean_anomaly.sin() + 1.25 * self.eccentricity.powi(2) * (2.0 * mean_anomaly).sin()
		}
	}
	impl Default for Orbit {
		fn default() -> Self {
			Self{ time: 0.0, semimajor_axis: 0.0, eccentricity: 0.0 }
		}
	}

	#[derive(Component)]
	pub struct FramerateCounter {
		pub frequency: f32,
		time: f32,
		measurements: Vec<f32>,
	}
	impl FramerateCounter {
		fn new(freq: f32) -> Self {
			Self{ frequency: freq, time: 0.0, measurements: Vec::new() }
		}
		/// Adds `delta` seconds to the counter's timer and returns the average framerate if enough
		/// seconds have elapsed
		fn add_time(&mut self, delta: f32) -> Option<f32> {
			self.time += delta;
			self.measurements.push(delta);
			if self.time >= self.frequency {
				let mut framerate_total = 0.0;
				for measurement in &self.measurements {
					let framerate = 1.0 / measurement;
					framerate_total += framerate;
				}
				let framerate_average = framerate_total / (self.measurements.len() as f32);
				self.time -= self.frequency;
				self.measurements.clear();
				return Some(framerate_average);
			} else {
				return None;
			}
		}
	}
	impl Default for FramerateCounter {
		fn default() -> Self {
			Self::new(0.3)
		}
	}

	pub fn setup_earth(
	    mut commands: Commands, asset_server: ResMut<AssetServer>,
	    mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>,
	) {
	    let earth: Body<f32> = Body::new_earth();
	    let equatorial_radius_engine = earth.radius_equator_km() * CONVERT_KM_TO_M * SCALE;
	    let polar_radius_engine = earth.radius_polar_km() * CONVERT_KM_TO_M * SCALE;
	    let mesh = Sphere::new(1.0).mesh().uv(200, 100);
	    let material = StandardMaterial{
	        base_color_texture: Some(asset_server.load("earth_albedo.jpeg")),
	        ..default()
	    };
	    info!("Engine planet radius: equatorial {} polar {}", equatorial_radius_engine, polar_radius_engine);
	    commands.spawn((
	        Mesh3d(meshes.add(mesh)),
	        MeshMaterial3d(materials.add(material)),
	        Transform::default()
	            .with_scale(Vec3::new(equatorial_radius_engine, polar_radius_engine, equatorial_radius_engine))
	            .with_rotation(Quat::from_euler(EulerRot::XYZ, -PI / 2.0, 0.0, PLANET_START_ROTATION)),
	        Planet,
			Wireframe,
	    ));
	}

	pub fn setup_sun(mut commands: Commands) {
	    let sun_direction: Dir3 = Dir3::new(Vec3::new(-1.0, -0.1, 0.2).normalize()).unwrap();
	    commands.spawn((
	        DirectionalLight::default(),
	        Transform::default().looking_to(sun_direction, Vec3::Y),
	    ));
	}

	pub fn setup_camera(mut commands: Commands) {
	    let earth: Body<f32> = Body::new_earth();
	    let planet_radius_km = earth.radius_equator_km();
	    let orbit_radius_km = planet_radius_km + ALTITUDE_KM;
	    let orbit_radius_engine = orbit_radius_km * CONVERT_KM_TO_M * SCALE;
	    info!("Engine orbit radius: {}", orbit_radius_engine);
	    commands.spawn((
	        Camera3d::default(),
	        Transform::from_xyz(orbit_radius_engine, 0.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
			Orbit::default().with_semimajor_axis(orbit_radius_km * CONVERT_KM_TO_M).with_eccentricity(ECCENTRICITY),
	    ));
	}

	pub fn setup_ui(mut commands: Commands){
		// framerate counter
		commands.spawn((
			Text::new("Framerate: 999.9 fr/s"),
			Node{
				position_type: PositionType::Absolute,
				right: Val::Px(6.0),
				top: Val::Px(2.0),
				..default()
			},
			FramerateCounter::default(),
		));
		// data display
		let orbital_parameters_header = commands.spawn(Text::new("Parameters")).id();
		let time = commands.spawn(Text::new("t: 1.234 s")).id();
		let semimajor_axis = commands.spawn(Text::new("a: 1234 km")).id();
		let eccentricity = commands.spawn(Text::new("e: 0.123")).id();
		let orbit_values_header = commands.spawn(Text::new("Values")).id();
		let mean_motion = commands.spawn(Text::new("n: 1.234")).id();
		let mean_anomaly = commands.spawn(Text::new("M: 0.123 rad")).id();
		let true_anomaly = commands.spawn(Text::new("nu: 0.123 rad")).id();
		commands.spawn(Node{
			position_type: PositionType::Absolute,
			left: Val::Px(6.0),
			bottom: Val::Px(2.0),
			flex_direction: FlexDirection::Row,
			column_gap: Val::Px(24.0),
			..default()
		}).with_children(|container|{
			container.spawn(Node{
				flex_direction: FlexDirection::Column,
				..default()
			}).add_child(orbital_parameters_header).add_child(time).add_child(semimajor_axis).add_child(eccentricity);
			container.spawn(Node{
				flex_direction: FlexDirection::Column,
				..default()
			}).add_child(orbit_values_header).add_child(mean_motion).add_child(mean_anomaly).add_child(true_anomaly);
		});
		commands.insert_resource(TextElements{ time, semimajor_axis, eccentricity, mean_motion, mean_anomaly, true_anomaly });
	}

	pub fn process_orbit(
		mut orbiting: Query<(&mut Transform, &mut Orbit), With<Camera3d>>,
		time: Res<Time>,
	){
		let delta = time.delta_secs();
		let earth = Body::new_earth();
		for (mut transform, mut orbit) in &mut orbiting {
			orbit.time += delta;
			let true_anomaly = orbit.true_anomaly(&earth);
			let radius = orbit.semimajor_axis * (1.0 - orbit.eccentricity.powi(2)) / (1.0 + orbit.eccentricity * true_anomaly.cos());
			let position = Quat::from_axis_angle(Vec3::Y, true_anomaly) * Vec3::X * (radius * SCALE);
			transform.translation = position;
		}
	}

	pub fn process_framerate_counter(
		mut text_nodes: Query<(&mut Text, &mut FramerateCounter)>,
		time: Res<Time>,
	){
		let delta = time.delta_secs();
		for (mut text, mut counter) in &mut text_nodes {
			if let Some(framerate) = counter.add_time(delta) {
				let message = format!("Framerate: {:.1} fr/s", framerate);
				text.0 = message;
			}
		}
	}

	pub fn process_display_text(
		mut text_nodes: Query<&mut Text>, orbits: Query<&Orbit>, labels: Res<TextElements>,
	){
		let body = Body::new_earth();
		let orbit = orbits.get_single().unwrap();
		let mut label = text_nodes.get_mut(labels.time).unwrap();
		label.0 = format!("t: {:.1} s", orbit.time);
		label = text_nodes.get_mut(labels.semimajor_axis).unwrap();
		label.0 = format!("a: {:.0} km", orbit.semimajor_axis * CONVERT_M_TO_KM);
		label = text_nodes.get_mut(labels.eccentricity).unwrap();
		label.0 = format!("e: {:.2}", orbit.eccentricity);
		label = text_nodes.get_mut(labels.mean_motion).unwrap();
		label.0 = format!("n: {}", orbit.mean_motion(&body));
		let mean_anomaly = orbit.mean_anomaly(&body);
		label = text_nodes.get_mut(labels.mean_anomaly).unwrap();
		label.0 = format!("M: {:.1}° ({:.4} rad)", mean_anomaly * CONVERT_RAD_TO_DEG, mean_anomaly);
		let true_anomaly = orbit.true_anomaly(&body);
		label = text_nodes.get_mut(labels.true_anomaly).unwrap();
		label.0 = format!("ν: {:.1}° ({:.4} rad)", true_anomaly * CONVERT_RAD_TO_DEG, true_anomaly);
	}

	pub fn render_gizmos(mut gizmos: Gizmos) {
		gizmos.axes(Transform::default(), 1.0);
	}
}

#[cfg(feature="example")]
use low_earth_orbit::*;

fn main() {
	#[cfg(feature="example")]
    {
    	App::new()
            .add_plugins((DefaultPlugins, WireframePlugin))
            .add_systems(Startup, (setup_earth, setup_camera, setup_sun, setup_ui))
            .add_systems(Update, (
    			process_orbit, process_framerate_counter,
    			process_display_text.after(process_orbit),
    			render_gizmos,
    		))
            .run();
    }
    #[cfg(not(feature="example"))]
    {
		println!("This example requires the `example` feature flag to be enabled. To run this example, run the following:");
		println!("\tcargo run --example low_earth_orbit --features example");
	}
}
