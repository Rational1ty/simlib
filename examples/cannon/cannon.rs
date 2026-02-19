use glam::DVec2;

#[derive(Clone, Debug, Default)]
pub struct Cannon {
	pub pos: DVec2,
	pub vel: DVec2,
	pub acc: DVec2,
}

impl Cannon {
	pub fn new(launch_angle: f64, launch_speed: f64) -> Self {
		let vx = launch_speed * launch_angle.cos();
		let vy = launch_speed * launch_angle.sin();

		Self {
			pos: DVec2::new(0.0, 0.0),
			vel: DVec2::new(vx, vy),
			acc: DVec2::new(0.0, -9.81),
		}
	}

	pub fn derivative(&self) -> Vec<f64> {
		vec![self.vel.x, self.vel.y, self.acc.x, self.acc.y]
	}
}
