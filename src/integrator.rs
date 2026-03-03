use crate::SimTime;

pub fn runge_kutta_4<S, D, L, U>(
	sim: &mut S,
	state_loader: &L,
	derivative: &D,
	state_unloader: &mut U,
	dt: f64,
	sim_time: &SimTime,
) where
	L: Fn(&mut S, &SimTime) -> Vec<f64>,
	D: Fn(&mut S, &SimTime) -> Vec<f64>,
	U: FnMut(&mut S, &[f64]),
{
	// we want to pass in the current integration dt to the functions, which might be different from
	// the simulation's default dt
	let t_0 = &SimTime { dt, ..*sim_time };
	let t_half = &SimTime {
		t: sim_time.t + (0.5 * dt),
		dt,
		..*sim_time
	};
	let t_full = &SimTime {
		t: sim_time.t + dt,
		dt,
		..*sim_time
	};

	let state = state_loader(sim, t_0);
	let n = state.len();

	let k1 = derivative(sim, t_0);

	let y2: Vec<f64> = (0..n).map(|i| state[i] + 0.5 * dt * k1[i]).collect();
	state_unloader(sim, &y2);
	let k2 = derivative(sim, t_half);

	let y3: Vec<f64> = (0..n).map(|i| state[i] + 0.5 * dt * k2[i]).collect();
	state_unloader(sim, &y3);
	let k3 = derivative(sim, t_half);

	let y4: Vec<f64> = (0..n).map(|i| state[i] + dt * k3[i]).collect();
	state_unloader(sim, &y4);
	let k4 = derivative(sim, t_full);

	let res: Vec<f64> = (0..n)
		.map(|i| state[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
		.collect();

	state_unloader(sim, &res);
}

pub struct Integrator<S> {
	pub state_loader: Box<dyn Fn(&mut S, &SimTime) -> Vec<f64>>,
	pub derivative: Box<dyn Fn(&mut S, &SimTime) -> Vec<f64>>,
	pub state_unloader: Box<dyn FnMut(&mut S, &[f64])>,
}
