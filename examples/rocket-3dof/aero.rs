//! Aerodynamic coefficient types for a rocket.

use crate::lut::Lut2;

#[derive(Clone, Debug, Default)]
pub struct BodyAeroCoefficients {
	pub cp: f64, // m, measured from nose tip
	pub cg: f64, // m, measured from nose tip
	pub surface_area: f64,
	pub ca_alpha_mach: Lut2,
	pub cn_alpha_mach: Lut2,
}
