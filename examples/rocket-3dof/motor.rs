//! Types for representing motor behavior.

use crate::lut::Lut1;

#[derive(Debug, Default)]
pub struct Motor {
	thrust_curve: Lut1,
}

impl Motor {
	pub fn get_thrust(&self, t: f64) -> f64 {
		self.thrust_curve.get(t)
	}
}
