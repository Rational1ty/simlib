use simlib::runge_kutta_4;

fn main() {
	// Test case: projectile motion
	// State: [x, y, vx, vy]
	// Derivatives: [vx, vy, 0, -9.81]

	let mut state = vec![0.0, 0.0, 10.0, 20.0]; // launch at angle
	let dt = 0.01;

	let derivative = |s: &[f64]| -> Vec<f64> { vec![s[2], s[3], 0.0, -9.81] };

	let mut t = 0.0;
	while state[1] >= 0.0 {
		state = runge_kutta_4(&state, derivative, dt);
		t += dt;
	}

	// Analytical solution for time of flight: 2 * vy0 / g = 2 * 20 / 9.81 ≈ 4.077s
	// Analytical solution for range: vx * t_flight = 10 * 4.077 ≈ 40.77m

	println!("Impact at t={:.3}s", t);
	println!("Final x={:.3}m", state[0]);
	println!("Expected: t≈4.077s, x≈40.77m");
}
