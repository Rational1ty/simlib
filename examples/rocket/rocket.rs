use glam::{DMat3, DQuat, DVec3};

use crate::{aero::BodyAeroCoefficients, atmosphere, motor::Motor};

enum FlightPhase {
	Boost,
	Coast,
	Ground,
}

pub struct Rocket {
	coeffs: BodyAeroCoefficients,
	position: DVec3,
	velocity: DVec3,
	acceleration: DVec3,
	orientation: DQuat,
	angular_vel: DVec3,
	angular_accel: DVec3,
	inertia: DMat3,
	flight_phase: FlightPhase,
	mass: f64,
	motor: Motor,
}

impl Rocket {
	pub fn get_aero_force_body(&self) -> DVec3 {
		let lcef_to_body_dcm = DMat3::from_quat(self.orientation).transpose();
		let v_body = lcef_to_body_dcm * self.velocity;

		let rho = atmosphere::get_air_density(self.position.x);
		let v = self.velocity.length();
		let s = self.coeffs.surface_area;

		// alpha is angle of attack; beta is sideslip angle (in body frame)
		let alpha = f64::atan2(v_body.z, v_body.x);
		let beta = f64::asin(v_body.y / v);

		let cx_alpha = self.coeffs.cx_alpha.get(alpha);
		let cy_beta = self.coeffs.cx_alpha.get(beta);
		let cz_alpha = self.coeffs.cx_alpha.get(alpha);

		let aero_load = 0.5 * rho * (v * v) * s;

		aero_load * DVec3::new(cx_alpha, cy_beta, cz_alpha)
	}

	pub fn derivative(&self) -> Vec<f64> {
		todo!();
	}

	pub fn integrate(&self) -> Vec<f64> {
		todo!();
	}
}
