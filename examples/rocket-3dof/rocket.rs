use glam::{DMat2, DVec2, dvec2};
use simlib::SimTime;

use crate::{aero::BodyAeroCoefficients, atmosphere, motor::Motor};

/// A model of a fin-stabilized rocket in two dimensions.
#[derive(Clone, Debug, Default)]
pub struct Rocket {
	pub coeffs: BodyAeroCoefficients,
	pub position: DVec2,     // LCEF m
	pub velocity: DVec2,     // LCEF m/s
	pub acceleration: DVec2, // LCEF m/s^2
	pub orientation: f64,    // rad, angle measured from horizontal (+x axis)
	pub angular_vel: f64,    // rad/s
	pub angular_accel: f64,  // rad/s^2
	pub inertia: f64,        // kg.m^2
	pub mass: f64,           // kg
	pub motor: Motor,
}

// AXIS CONVENTIONS:
//   LCEF (inertial/world): x right, y up
//   body (rocket frame): x along nose, y to the left of nose
//   z out of screen for RHR
//   orientation: 0 along LCEF +x axis, CCW is positive
impl Rocket {
	fn get_aero_force_body(&self) -> DVec2 {
		let lcef_to_body_dcm = DMat2::from_angle(self.orientation).transpose();
		let vel_body = lcef_to_body_dcm * self.velocity;
		// eprintln!("vel_body = {:?}", vel_body);
		let v = self.velocity.length();

		// no velocity => no aero forces
		if v < 0.1 {
			return DVec2::ZERO;
		}

		let alpha = -f64::atan2(vel_body.y, vel_body.x);
		// assert!(
		// 	alpha.abs() < 10_f64.to_radians(),
		// 	"AOA exceeded allowed range: α={alpha} v_body={vel_body}"
		// );
		let mach = velocity_to_mach(v, self.position.y);
		// assert!(mach < 3.0, "mach was out of range: M={mach} v={v} y={}", self.position.y);

		// approximations for small AOA
		let ca = self.coeffs.ca_mach.get(mach);
		let cn = self.coeffs.cn_alpha_mach.get(mach) * alpha;

		let rho = atmosphere::get_air_density(self.position.y);
		let s = self.coeffs.surface_area;

		let aero_load = 0.5 * rho * (v * v) * s;

		aero_load * dvec2(-ca, cn)
	}

	pub fn derivative(&self, time: &SimTime) -> Vec<f64> {
		let body_to_lcef_dcm = DMat2::from_angle(self.orientation);

		// translational forces
		let thrust_body = dvec2(self.motor.get_thrust(time.t), 0.0);

		let aero_force_body = self.get_aero_force_body();
		let net_force_body = thrust_body + aero_force_body;

		let gravity_accel = dvec2(0.0, -9.81);
		let mass = self.mass + self.motor.total_weight_kg
			- (self.motor.prop_weight_kg * (time.t / self.motor.burn_time_end).min(1.0));
		let net_acceleration_body = net_force_body / mass;
		let net_acceleration_lcef = (body_to_lcef_dcm * net_acceleration_body) + gravity_accel;

		let normal_force = aero_force_body.y;
		let net_moment = -normal_force * (self.coeffs.cp - self.coeffs.cg);
		let net_angular_accel = net_moment / self.inertia;

		vec![
			self.velocity.x,
			self.velocity.y,
			net_acceleration_lcef.x,
			net_acceleration_lcef.y,
			self.angular_vel,
			net_angular_accel,
		]
	}
}

fn velocity_to_mach(vel: f64, altitude_m: f64) -> f64 {
	const GAMMA: f64 = 1.4; // adiabatic index of air
	const R_AIR: f64 = 287.05; // ideal gas constant for air

	let temperature_degc = atmosphere::get_temperature(altitude_m);
	let temperature_k = temperature_degc + 273.15;

	let speed_of_sound = f64::sqrt(GAMMA * R_AIR * temperature_k);

	vel / speed_of_sound
}
