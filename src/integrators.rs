pub fn runge_kutta_4<F>(state: &[f64], dt: f64, derivative: F) -> Vec<f64>
where
	F: Fn(&[f64]) -> Vec<f64>,
{
	let n = state.len();

	let k1 = derivative(state);

	let y2: Vec<f64> = (0..n).map(|i| state[i] + 0.5 * dt * k1[i]).collect();
	let k2 = derivative(&y2);

	let y3: Vec<f64> = (0..n).map(|i| state[i] + 0.5 * dt * k2[i]).collect();
	let k3 = derivative(&y3);

	let y4: Vec<f64> = (0..n).map(|i| state[i] + dt * k3[i]).collect();
	let k4 = derivative(&y4);

	(0..n)
		.map(|i| state[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
		.collect()
}
