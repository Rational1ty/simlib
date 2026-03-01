//! Types for representing motor behavior.

use std::{fs, io, iter, path::Path};

use crate::lut::Lut1;

#[derive(Clone, Debug, Default)]
pub struct Motor {
	pub designation: String,
	pub prop_weight_kg: f64,
	pub total_weight_kg: f64,
	pub burn_time_end: f64,
	pub thrust_curve: Lut1,
}

impl Motor {
	pub fn from_eng_file<P: AsRef<Path>>(file_path: P) -> io::Result<Self> {
		let file_string = fs::read_to_string(file_path)?;
		Ok(Self::parse_eng_file_str(&file_string))
	}

	fn parse_eng_file_str(contents: &str) -> Self {
		// get file lines and remove comments
		let lines: Vec<&str> = contents
			.lines()
			.map(str::trim)
			.filter(|line| !line.starts_with(";"))
			.collect();

		assert!(lines.len() > 1, ".eng file must have a header line and at least one thrust point");

		let first_line: Vec<&str> = lines.first().unwrap().split_ascii_whitespace().collect();

		let designation = first_line[0].to_string();
		let prop_weight_kg = first_line[4]
			.parse::<f64>()
			.expect("should be a valid float");
		let total_weight_kg = first_line[5]
			.parse::<f64>()
			.expect("should be a valid float");

		let points_iter = lines[1..].iter().map(|line| {
			let (a, b) = split_once_whitespace(line).expect("line should contain two values");
			let time = a.parse::<f64>().expect("should be a valid float");
			let thrust = b.parse::<f64>().expect("should be a valid float");
			(time, thrust)
		});

		let (time_pts, thrust_pts): (Vec<_>, Vec<_>) =
			iter::once((0.0, 0.0)).chain(points_iter).unzip();

		assert!(*thrust_pts.last().unwrap() == 0.0, "final point must have a thrust of zero");

		let max_thrust = *thrust_pts.iter().max_by(|a, b| a.total_cmp(b)).unwrap();
		let end_of_thrust = thrust_pts.len() - 1 - thrust_pts
			.iter()
			.rev()
			.position(|thrust| *thrust <= 0.05 * max_thrust)
			.unwrap();
		let burn_time_end = time_pts[end_of_thrust];

		let thrust_curve = Lut1::new(&time_pts, &thrust_pts);

		Self {
			designation,
			prop_weight_kg,
			total_weight_kg,
			burn_time_end,
			thrust_curve,
		}
	}

	pub fn get_thrust(&self, t: f64) -> f64 {
		self.thrust_curve.saturating_get(t)
	}
}

fn split_once_whitespace(s: &str) -> Option<(&str, &str)> {
	let (a, b) = s.split_once(|c: char| c.is_ascii_whitespace())?;
	let b = b.trim_ascii_start();
	Some((a, b))
}

#[cfg(test)]
mod tests {
	use super::*;
	use dedent::dedent;

	#[test]
	fn parses_eng_file_from_string() {
		#[rustfmt::skip]
		let eng = dedent!("
			; Rocketvision F32
			; from NAR data sheet updated 11/2000
			; created by John Coker 5/2006
			F32 24 124 5-10-15 .0377 .0695 RV
			0.01 50
			0.05 56
			0.10 48
			2.00 24
			2.20 19
			2.24  5
			2.72  0
			;
		");

		let motor = Motor::parse_eng_file_str(eng);
		println!("{:?}", motor);

		assert_eq!(motor.designation, "F32");
		assert_eq!(motor.prop_weight_kg, 0.0377);
		assert_eq!(motor.total_weight_kg, 0.0695);
		assert_eq!(motor.burn_time_end, 2.72);
		assert_eq!(motor.get_thrust(0.01), 50.0);
	}
}
