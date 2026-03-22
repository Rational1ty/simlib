//! Aerodynamic coefficient types for a rocket.

use crate::lut::Lut2;

#[derive(Clone, Debug, Default)]
pub struct BodyAeroCoefficients {
	pub cp: f64, // m from nose tip
	pub cg: f64, // m from nose tip
	pub surface_area: f64,
	pub cx_alpha_mach: Lut2, // C_x(alpha, mach)
	pub cy_beta_mach: Lut2,  // C_y(beta, mach)
	pub cz_alpha_mach: Lut2, // C_z(alpha, mach)
}
