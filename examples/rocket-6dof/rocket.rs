use glam::{DQuat, DVec3, dvec3};
use simlib::SimTime;

use crate::{aero::BodyAeroCoefficients, atmosphere, motor::Motor};

#[derive(Clone, Debug, Default)]
pub struct Rail {
	pub angle: f64,  // rad
	pub length: f64, // m
}

impl Rail {
	pub fn direction(&self) -> DVec3 {
		dvec3(self.angle.cos(), 0.0, self.angle.sin())
	}

	pub fn initial_orientation(&self) -> DQuat {
		DQuat::from_rotation_y(-self.angle)
	}
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum FlightPhase {
	#[default]
	OnRail,
	Boost,
	Coast,
	Descent,
	Ground,
}

/// A model of a fin-stabilized rocket in three dimensions.
#[derive(Clone, Debug, Default)]
pub struct Rocket {
	pub coeffs: BodyAeroCoefficients,
	pub position: DVec3,     // ENU m
	pub velocity: DVec3,     // ENU m/s
	pub acceleration: DVec3, // ENU m/s^2
	pub orientation: DQuat,  // body to ENU
	pub angular_vel: DVec3,  // body rad/s
	pub angular_accel: DVec3, // body rad/s^2
	pub inertia: DVec3,      // kg.m^2, principal moments
	pub mass: f64,           // kg
	pub flight_phase: FlightPhase,
	pub motor: Motor,
	pub rail: Rail,
}

// AXIS CONVENTIONS:
//   ENU (inertial/world): x east, y north, z up
//   body (rocket frame): +x along nose axis
impl Rocket {
	fn get_aero_force_body(&self) -> DVec3 {
		let body_to_enu = self.orientation;
		let vel_body = body_to_enu.conjugate() * self.velocity;
		let v = vel_body.length();

		if v < 0.1 {
			return DVec3::ZERO;
		}

		let rho = atmosphere::get_air_density(self.position.z.max(0.0));
		let q_s = 0.5 * rho * self.coeffs.surface_area;

		dvec3(
			-q_s * self.coeffs.c_x * vel_body.x.abs() * vel_body.x,
			-q_s * self.coeffs.c_y * vel_body.y.abs() * vel_body.y,
			-q_s * self.coeffs.c_z * vel_body.z.abs() * vel_body.z,
		)
	}

	pub fn derivative(&mut self, time: &SimTime) -> Vec<f64> {
		let body_to_enu = self.orientation;

		let gravity_accel = dvec3(0.0, 0.0, -9.81);

		let thrust_body = dvec3(self.motor.get_thrust(time.t), 0.0, 0.0);
		let thrust_enu = body_to_enu * thrust_body;

		let aero_force_body = self.get_aero_force_body();
		let aero_force_enu = body_to_enu * aero_force_body;

		let mass = self.mass + self.motor.total_weight_kg;
		let net_acceleration_enu = ((thrust_enu + aero_force_enu) / mass) + gravity_accel;

		if self.flight_phase == FlightPhase::OnRail {
			let rail_dir = self.rail.direction();
			let pos_along_rail = self.position.dot(rail_dir);

			if pos_along_rail < self.rail.length {
				let accel_parallel = net_acceleration_enu.dot(rail_dir).max(0.0);
				let accel_on_rail = rail_dir * accel_parallel;
				self.acceleration = accel_on_rail;
				self.angular_accel = DVec3::ZERO;

				return vec![
					self.velocity.x,
					self.velocity.y,
					self.velocity.z,
					accel_on_rail.x,
					accel_on_rail.y,
					accel_on_rail.z,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
					0.0,
				];
			}

			self.flight_phase = FlightPhase::Boost;
		}

		self.acceleration = net_acceleration_enu;
		self.angular_accel = DVec3::ZERO;

		vec![
			self.velocity.x,
			self.velocity.y,
			self.velocity.z,
			net_acceleration_enu.x,
			net_acceleration_enu.y,
			net_acceleration_enu.z,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
		]
	}
}

pub fn velocity_to_mach(vel: f64, altitude_m: f64) -> f64 {
	const GAMMA: f64 = 1.4; // adiabatic index of air
	const R_AIR: f64 = 287.05; // ideal gas constant for air

	let temperature_degc = atmosphere::get_temperature(altitude_m);
	let temperature_k = temperature_degc + 273.15;

	let speed_of_sound = f64::sqrt(GAMMA * R_AIR * temperature_k);

	vel / speed_of_sound
}
