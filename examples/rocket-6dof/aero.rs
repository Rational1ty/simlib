//! Aerodynamic coefficient types for a rocket.

#[derive(Clone, Debug, Default)]
pub struct BodyAeroCoefficients {
	pub surface_area: f64,
	pub c_x: f64,
	pub c_y: f64,
	pub c_z: f64,
}
