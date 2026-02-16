use simlib::{Executor, Phase, Recorder};

#[derive(Clone, Debug, Default)]
struct Simulation {
	position: (f64, f64),
	velocity: (f64, f64),
}

fn main() {
	let sim = Simulation {
		position: (0.0, 0.0),
		velocity: (10.0, 20.0),
	};

	let dt = 0.1;
	let end_time = 5.0;

	let mut recorder = Recorder::<Simulation>::new("test-sim.csv");
	recorder.track("position_x", |sim| sim.position.0);
	recorder.track("position_y", |sim| sim.position.1);
	recorder.track("velocity_x", |sim| sim.velocity.0);
	recorder.track("velocity_y", |sim| sim.velocity.1);

	let mut exec = Executor::<Simulation>::new(dt, end_time);
	exec.set_recorder(recorder);

	exec.add_job(Phase::Init, |sim, time| {
		println!("Starting simulation at t={} with sim={:?}", time.t, sim);
	});

	exec.set_integrator(
		|sim| {
			vec![
				sim.position.0,
				sim.position.1,
				sim.velocity.0,
				sim.velocity.1,
			]
		},
		|s| vec![s[2], s[3], 0.0, -9.81],
		|sim, s| {
			sim.position = (s[0], s[1]);
			sim.velocity = (s[2], s[3]);
		},
	);

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

	exec.run(sim);

	println!("Recorded variables saved to output.csv");
}
