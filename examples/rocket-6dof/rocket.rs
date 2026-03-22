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
	pub position: DVec3,      // ENU m
	pub velocity: DVec3,      // ENU m/s
	pub acceleration: DVec3,  // ENU m/s^2
	pub orientation: DQuat,   // body to ENU
	pub angular_vel: DVec3,   // body rad/s
	pub angular_accel: DVec3, // body rad/s^2
	pub inertia: DVec3,       // kg.m^2, principal moments
	pub mass: f64,            // kg
	pub flight_phase: FlightPhase,
	pub motor: Motor,
	pub rail: Rail,
}

// AXIS CONVENTIONS:
//   ENU (inertial/world): x east, y north, z up
//   body (rocket frame): +x along nose axis
impl Rocket {
	/// Computes `q̇ = (1/2) q ⨂ ω_q`,
	/// where
	/// - `q` is the rocket's orientation quaternion
	/// - `ω_q` is the rocket's angular rate as a quaternion `[ω, 0]`
	fn quaternion_derivative(body_to_enu: DQuat, omega_body: DVec3) -> DQuat {
		let qv = body_to_enu.xyz();
		let qv_dot = 0.5 * ((body_to_enu.w * omega_body) + qv.cross(omega_body));
		let qw_dot = -0.5 * qv.dot(omega_body);

		DQuat::from_xyzw(qv_dot.x, qv_dot.y, qv_dot.z, qw_dot)
	}

	/// Solves for `ω_dot` in the equation `M = Iω_dot + ω ⨉ (Iω)`.
	fn angular_accel_body(&self, net_moment_body: DVec3) -> DVec3 {
		let inertia = self.inertia;
		let omega = self.angular_vel;
		let i_omega = self.inertia * self.angular_vel;
		let coriolis = omega.cross(i_omega);

		dvec3(
			(net_moment_body.x - coriolis.x) / inertia.x,
			(net_moment_body.y - coriolis.y) / inertia.y,
			(net_moment_body.z - coriolis.z) / inertia.z,
		)
	}

	fn get_aero_force_body(&self) -> DVec3 {
		let body_to_enu = self.orientation;
		let vel_body = body_to_enu.conjugate() * self.velocity;
		let v = vel_body.length();

		if v < 0.1 {
			return DVec3::ZERO;
		}

		let rho = atmosphere::get_air_density(self.position.z.max(0.0));
		let q_s = 0.5 * rho * v * v * self.coeffs.surface_area;

		let alpha = f64::atan2(-vel_body.z, vel_body.x);
		let beta = f64::atan2(vel_body.y, f64::hypot(vel_body.x, vel_body.z));
		let mach = velocity_to_mach(v, self.position.z.max(0.0));

		let c_x = self.coeffs.cx_alpha_mach.get(alpha.abs(), mach);
		let c_y = self.coeffs.cy_beta_mach.get(beta.abs(), mach);
		let c_z = self.coeffs.cz_alpha_mach.get(alpha.abs(), mach);

		let force_x = -q_s * c_x;
		let force_y = -q_s * c_y * beta.signum();
		let force_z = -q_s * c_z * alpha.signum();

		dvec3(force_x, force_y, force_z)
	}

	pub fn derivative(&mut self, time: &SimTime) -> Vec<f64> {
		if self.flight_phase == FlightPhase::Ground {
			self.acceleration = DVec3::ZERO;
			self.angular_accel = DVec3::ZERO;
			return vec![0.0; 13];
		}

		let body_to_enu = self.orientation;

		let gravity_accel = dvec3(0.0, 0.0, -9.81);

		let thrust_body = dvec3(self.motor.get_thrust(time.t), 0.0, 0.0);
		let thrust_enu = body_to_enu * thrust_body;

		let aero_force_body = self.get_aero_force_body();
		let aero_force_enu = body_to_enu * aero_force_body;

		let moment_arm = dvec3(self.coeffs.cp - self.coeffs.cg, 0.0, 0.0);
		let aero_moment_body = moment_arm.cross(aero_force_body);
		let net_angular_accel_body = self.angular_accel_body(aero_moment_body);

		let quat_dot = Self::quaternion_derivative(body_to_enu, self.angular_vel);

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
		self.angular_accel = net_angular_accel_body;

		vec![
			self.velocity.x,
			self.velocity.y,
			self.velocity.z,
			net_acceleration_enu.x,
			net_acceleration_enu.y,
			net_acceleration_enu.z,
			quat_dot.x,
			quat_dot.y,
			quat_dot.z,
			quat_dot.w,
			net_angular_accel_body.x,
			net_angular_accel_body.y,
			net_angular_accel_body.z,
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
