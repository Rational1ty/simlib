//! Simple lookup table implementations.

/// A one-dimensional lookup table.
#[derive(Debug, Default)]
pub struct Lut1 {
	// TODO: use NonEmptyVec
	ts: Vec<f64>,
	data: Vec<f64>,
}

impl Lut1 {
	pub fn new(ts: &[f64], data: &[f64]) -> Self {
		assert!(ts.len() == data.len());
		Self {
			ts: ts.to_vec(),
			data: data.to_vec(),
		}
	}

	pub fn get(&self, t: f64) -> f64 {
		if t < self.ts[0] || t > self.ts[self.ts.len() - 1] {
			panic!("requested value {t} is outside Lut1 range");
		}

		// find first index with ts[idx] >= t
		let hi = self.ts.partition_point(|&x| x < t);

		if hi == 0 {
			return self.data[0];
		}

		let lo = hi - 1;
		let t0 = self.ts[lo];
		let t1 = self.ts[hi];
		let y0 = self.data[lo];
		let y1 = self.data[hi];

		// ensure no division by zero
		if t0 == t1 {
			return y0;
		}

		let t_fraction = (t - t0) / (t1 - t0);

		y0 + t_fraction * (y1 - y0)
	}
}

/// A two-dimensional lookup table.
#[derive(Debug, Default)]
pub struct Lut2 {
	dim: (usize, usize), // the dimensions of the LUT (# points along each axis)
	ts: Vec<f64>,        // coordinates along dimension 0
	us: Vec<f64>,        // coordinates along dimension 1
	data: Vec<f64>,      // flattened data array
}

impl Lut2 {
	pub fn new(ts: &[f64], us: &[f64], data: &[f64]) -> Self {
		assert!(ts.len() * us.len() == data.len());
		Self {
			dim: (ts.len(), us.len()),
			ts: ts.to_vec(),
			us: ts.to_vec(),
			data: ts.to_vec(),
		}
	}

	pub fn get(&self, t: f64, u: f64) -> f64 {
		if t < self.ts[0] || t > self.ts[self.ts.len() - 1] {
			panic!("requested value {t} is outside Lut2 range");
		}
		if u < self.us[0] || u > self.us[self.us.len() - 1] {
			panic!("requested value {u} is outside Lut2 range");
		}

		// find indicies along t and u axes
		let (lo_t, hi_t, alpha_t) = Self::get_axis_indicies(t, &self.ts);
		let (lo_u, hi_u, alpha_u) = Self::get_axis_indicies(u, &self.us);

		// function that gets values (row-major: i * dim1 + j)
		let corner = |i: usize, j: usize| self.data[i * self.dim.1 + j];

		let v00 = corner(lo_t, lo_u);
		let v10 = corner(hi_t, lo_u);
		let v01 = corner(lo_t, hi_u);
		let v11 = corner(hi_t, hi_u);

		// bilinear interpolation
		let v0 = v00 + alpha_t * (v10 - v00);
		let v1 = v01 + alpha_t * (v11 - v01);

		v0 + alpha_u * (v1 - v0)
	}

	fn get_axis_indicies(value: f64, axis: &[f64]) -> (usize, usize, f64) {
		let hi = axis.partition_point(|&x| x < value);

		if hi == 0 {
			return (0, 0, 0.0);
		}

		let lo = hi - 1;
		let v0 = axis[lo];
		let v1 = axis[hi];

		let alpha = if v0 == v1 {
			0.0
		} else {
			(value - v0) / (v1 - v0)
		};

		(lo, hi, alpha)
	}
}
