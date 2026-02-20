//! Aerodynamic coefficient types for a rocket.

use crate::lut::Lut1;

#[derive(Debug, Default)]
pub struct BodyAeroCoefficients {
	pub cp: f64, // m, measured from nose tip
	pub cg: f64, // m, measured from nose tip
	pub surface_area: f64,
	pub cn_mach: Lut1,
	pub ca_mach: Lut1,
}
