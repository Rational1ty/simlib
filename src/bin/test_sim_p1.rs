use simlib::{Executor, Phase};

#[derive(Debug)]
struct Simulation {
	position: f64,
	velocity: f64,
}

fn main() {
	let mut sim = Simulation {
		position: 0.0,
		velocity: 10.0,
	};

	let dt = 0.1;
	let end_time = 2.0;

	let mut exec = Executor::<Simulation>::new(dt, end_time);

	exec.add_job(Phase::Init, |sim, time| {
		println!("Starting simulation at t={} with sim={:?}", time.t, sim);
	});

	exec.add_job(Phase::Integrate, |sim, time| {
		// Simple Euler integration for test
		sim.position += sim.velocity * time.dt;
		sim.velocity += -9.81 * time.dt; // gravity
	});

	exec.add_job(Phase::PostIntegrate, |sim, time| {
		println!(
			"t={:.2}: pos={:.3}, vel={:.3}",
			time.t, sim.position, sim.velocity
		);
	});

	exec.add_job(Phase::Shutdown, |sim, time| {
		println!("Simulation complete. Final position: {:.3} at {:.3} s", sim.position, time.t);
	});

	exec.run(&mut sim);
}
