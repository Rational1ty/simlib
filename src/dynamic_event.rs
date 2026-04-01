use crate::SimTime;

pub trait DynamicEvent<S> {
	fn time_to_go(&mut self, sim: &S, time: f64) -> Option<f64>;
	fn apply(&mut self, sim: &mut S, time: &SimTime);
}

#[derive(Debug, PartialEq, Eq)]
pub enum CrossingMode {
	Any,
	Increasing,
	Decreasing,
}

pub struct RegulaFalsi<S> {
	lower_bound: Option<(f64, f64)>,
	upper_bound: Option<(f64, f64)>,
	tol: f64,
	mode: CrossingMode,
	error_fn: Box<dyn Fn(&S) -> f64>,
	action: Box<dyn FnMut(&mut S, &SimTime)>,
	iterations: u32,
}

// sim -> error
// error -> tgo
// tgo -> iterate
// tgo == 0 -> apply
impl<S> RegulaFalsi<S> {
	pub fn new<E, F>(mode: CrossingMode, error_fn: E, action: F) -> Self
	where
		E: Fn(&S) -> f64 + 'static,
		F: FnMut(&mut S, &SimTime) + 'static,
	{
		Self {
			// prev_t_err: None,
			lower_bound: None,
			upper_bound: None,
			tol: 1e-12,
			mode,
			error_fn: Box::new(error_fn),
			action: Box::new(action),
			iterations: 0,
		}
	}

	pub fn reset(&mut self) {
		self.lower_bound = None;
		self.upper_bound = None;
		self.iterations = 0;
	}
}

// Note: Because regula falsi uses an iteration counter, `time_to_go` is technically not idempotent
// as you would expect it to be. But in practice, if the max iteration count is high enough, this is
// not a problem.
impl<S> DynamicEvent<S> for RegulaFalsi<S> {
	fn time_to_go(&mut self, sim: &S, time: f64) -> Option<f64> {
		let t = time;
		let f = (self.error_fn)(sim);

		let Some((t_lo, f_lo)) = self.lower_bound else {
			self.lower_bound = Some((t, f));
			return None;
		};

		if self.upper_bound.is_none() {
			// In any of these cases, a valid crossing hasn't occurred, so we
			// can't set the upper bound yet.
			if f_lo.signum() == f.signum() {
				self.lower_bound = Some((t, f));
				return None;
			}
			if self.mode == CrossingMode::Increasing && f.signum() < 0.0 {
				self.lower_bound = Some((t, f));
				return None;
			}
			if self.mode == CrossingMode::Decreasing && f.signum() > 0.0 {
				self.lower_bound = Some((t, f));
				return None;
			}

			self.upper_bound = Some((t, f));
			let c = ((t_lo * f) - (t * f_lo)) / (f - f_lo);
			self.iterations += 1;
			return Some(c - t);
		}

		if f.abs() < self.tol {
			return Some(0.0);
		}

		// If we get here, then we are in the refinement phase.

		if self.iterations >= 50 {
			return Some(0.0);
		}

		// If the endpoint hasn't moved (no refinement) then this doesn't count as an iteration
		// and we just return the same tgo as last time.
		let (t_hi, f_hi) = self.upper_bound.unwrap();
		if t == t_lo || t == t_hi {
			let c = ((t_lo * f_hi) - (t_hi * f_lo)) / (f_hi - f_lo);
			println!("got here");
			return Some(c - t);
		}

		if f.signum() == f_lo.signum() {
			self.lower_bound = Some((t, f));
		} else {
			self.upper_bound = Some((t, f));
		}
		self.iterations += 1;

		let (t_lo, f_lo) = self.lower_bound.unwrap();
		let (t_hi, f_hi) = self.upper_bound.unwrap();

		let c = ((t_lo * f_hi) - (t_hi * f_lo)) / (f_hi - f_lo);
		Some(c - t)
	}

	fn apply(&mut self, sim: &mut S, time: &SimTime) {
		(self.action)(sim, time);
	}
}

#[cfg(test)]
mod tests {
	use glam::DVec2;

	use super::*;
	use crate::{integrator::Integrator, runge_kutta_4};

	#[derive(Clone, Debug)]
	struct CannonSim {
		position: DVec2,
		velocity: DVec2,
	}

	#[test]
	fn regula_falsi_any() {
		const TOL: f64 = 1e-10;
		const MAGIC_ALT: f64 = 50.0;
		const LAUNCH_VEL: f64 = 50.0;
		const LAUNCH_ANGLE: f64 = 60_f64.to_radians();

		let mut sim = CannonSim {
			position: DVec2::ZERO,
			velocity: LAUNCH_VEL * DVec2::from_angle(LAUNCH_ANGLE),
		};

		let mut rf = RegulaFalsi::<CannonSim>::new(
			CrossingMode::Any,
			|sim| sim.position.y - MAGIC_ALT,
			|sim, time| {
				println!("[EVENT RESULT] t={} pos_y={}", time.t, sim.position.y);

				assert!((sim.position.y - MAGIC_ALT).abs() < TOL);
				assert!(
					(time.t - 1.3660996344312).abs() < TOL
						|| (time.t - 7.4618861329942).abs() < TOL
				);
			},
		);

		let integ = get_integrator(
			|sim, _| {
				vec![
					sim.position.x,
					sim.position.y,
					sim.velocity.x,
					sim.velocity.y,
				]
			},
			|sim, _| vec![sim.velocity.x, sim.velocity.y, 0.0, -9.81],
			|sim, s| {
				sim.position.x = s[0];
				sim.position.y = s[1];
				sim.velocity.x = s[2];
				sim.velocity.y = s[3];
			},
		);

		let end_time = 10.0;
		let default_dt = 0.1;
		let mut step = 0;

		let mut t_from = 0.0;
		let mut t_to = t_from + default_dt;

		'outer: while t_from < end_time {
			// let saved_sim_state = sim.clone();
			let dt = t_to - t_from;
			runge_kutta_4(
				&mut sim,
				&integ.state_loader,
				&integ.derivative,
				&integ.state_unloader,
				dt,
				&SimTime {
					t: t_from,
					dt,
					step,
				},
			);

			println!(
				"t={:.2} pos=({:.3}, {:.3}) vel=({:.3}, {:.3})",
				t_to, sim.position.x, sim.position.y, sim.velocity.x, sim.velocity.y
			);

			let mut t_dyn = t_to;

			while let Some(tgo) = rf.time_to_go(&sim, t_dyn) {
				if tgo == 0.0 {
					rf.apply(&mut sim, &SimTime { t: t_dyn, dt, step });
					rf.reset();
					t_from = t_dyn;
					t_to = default_dt * (step + 1) as f64;
					println!("Done with event; setting t_from={t_from} t_to={t_to}");
					continue 'outer;
				}

				// for _ in 0..5 {
				// 	rf.time_to_go(&sim, &SimTime { t: t_dyn, dt, step });
				// 	println!(
				// 		"called time_to_go from {t_from} to {t_dyn}, rf.lb={:?}, rf.ub={:?}",
				// 		rf.lower_bound, rf.upper_bound
				// 	);
				// }

				// sim = saved_sim_state.clone();
				t_from = t_dyn;
				t_dyn += tgo;
				let integ_dt = t_dyn - t_from;
				println!(
					"[DYN-EVENT] tgo={} integrating from {} to {}, dt={}",
					tgo, t_from, t_dyn, integ_dt
				);
				runge_kutta_4(
					&mut sim,
					&integ.state_loader,
					&integ.derivative,
					&integ.state_unloader,
					integ_dt,
					&SimTime {
						t: t_from,
						dt: integ_dt,
						step: step,
					},
				);
			}

			step += 1;
			t_from = default_dt * step as f64;
			t_to = default_dt * (step + 1) as f64;
		}
	}

	fn get_integrator<L, D, U>(
		state_loader: L,
		derivative: D,
		state_unloader: U,
	) -> Integrator<CannonSim>
	where
		L: Fn(&mut CannonSim, &SimTime) -> Vec<f64> + 'static,
		D: Fn(&mut CannonSim, &SimTime) -> Vec<f64> + 'static,
		U: Fn(&mut CannonSim, &[f64]) + 'static,
	{
		Integrator {
			state_loader: Box::new(state_loader),
			derivative: Box::new(derivative),
			state_unloader: Box::new(state_unloader),
		}
	}
}
