use simlib::{Executor, Phase, Recorder, runge_kutta_4};

#[derive(Clone, Debug, Default)]
struct Simulation {
	position: (f64, f64),
	velocity: (f64, f64),
}

fn main() {
	let mut sim = Simulation {
		position: (0.0, 0.0),
		velocity: (10.0, 20.0),
	};

	let dt = 0.1;
	let end_time = 5.0;

	let mut recorder = Recorder::<Simulation>::new();
	recorder.track("position_x", |sim| sim.position.0);
	recorder.track("position_y", |sim| sim.position.1);
	recorder.track("velocity_x", |sim| sim.velocity.0);
	recorder.track("velocity_y", |sim| sim.velocity.1);

	let mut exec = Executor::<Simulation>::with_recorder(dt, end_time, recorder);

	exec.add_job(Phase::Init, |sim, time| {
		println!("Starting simulation at t={} with sim={:?}", time.t, sim);
	});

	exec.add_job(Phase::Integrate, |sim, time| {
		// Simple Euler integration for test
		let state = &[
			sim.position.0,
			sim.position.1,
			sim.velocity.0,
			sim.velocity.1,
		];
		let res = runge_kutta_4(state, time.dt, |s| vec![s[2], s[3], 0.0, -9.81]);
		sim.position = (res[0], res[1]);
		sim.velocity = (res[2], res[3]);
	});

	exec.add_job(Phase::PostIntegrate, |sim, time| {
		println!(
			"t={:.2}: pos=({:.2}, {:.2}), vel=({:.2}, {:.2})",
			time.t, sim.position.0, sim.position.1, sim.velocity.0, sim.velocity.1
		);
	});

	exec.add_job(Phase::Shutdown, |sim, time| {
		println!(
			"Simulation complete. Final position: ({:.2}, {:.2}) at {:.3} s",
			sim.position.0, sim.position.1, time.t
		);
	});

	exec.run(&mut sim);

	println!("Recorded variables saved to output.csv");
}
