//! Aerodynamic coefficient types for a rocket.

#[derive(Clone, Debug, Default)]
pub struct BodyAeroCoefficients {
	pub cp: f64, // m from nose tip
	pub cg: f64, // m from nose tip
	pub surface_area: f64,
	pub c_x: f64,
	pub c_y: f64,
	pub c_z: f64,
}
