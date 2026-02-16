//! Aerodynamic coefficient types for a rocket.

use crate::lut::Lut1;

#[derive(Debug, Default)]
pub struct BodyAeroCoefficients {
	pub cp: f64,
	pub cg: f64,
	pub surface_area: f64,
	pub cx_alpha: Lut1,
	pub cy_beta: Lut1,
	pub cz_alpha: Lut1,
}
