use std::f64::consts::PI;

use simlib::{Executor, Phase, Recorder};

use crate::cannon::Cannon;

mod cannon;

#[derive(Clone, Debug, Default)]
struct CannonSim {
	pub cannon: Cannon,
}

fn main() {
	let cannon_sim = CannonSim {
		cannon: Cannon::new(PI / 3.0, 50.0),
	};

	let dt = 0.1;
	let end_time = 10.0;
	let mut exec = Executor::<CannonSim>::new(dt, end_time);

	let filename = "cannon-sim.csv";
	let mut recorder = Recorder::<CannonSim>::new(filename);
	recorder.track("pos_x", |sim| sim.cannon.pos.x);
	recorder.track("pos_y", |sim| sim.cannon.pos.x);
	recorder.track("vel_x", |sim| sim.cannon.vel.x);
	recorder.track("vel_y", |sim| sim.cannon.vel.x);
	recorder.track("acc_x", |sim| sim.cannon.acc.x);
	recorder.track("acc_y", |sim| sim.cannon.acc.x);

	exec.set_recorder(recorder);

	exec.set_integrator(
		|sim| {
			vec![
				sim.cannon.pos.x,
				sim.cannon.pos.y,
				sim.cannon.vel.x,
				sim.cannon.vel.y,
			]
		},
		|sim| sim.cannon.derivative(),
		|sim, s| {
			sim.cannon.pos.x = s[0];
			sim.cannon.pos.y = s[1];
			sim.cannon.vel.x = s[2];
			sim.cannon.vel.y = s[3];
		},
	);

	exec.add_job(Phase::Init, |_, _| {
		println!("Starting cannon sim");
	});

	exec.add_job(Phase::PostIntegrate, |sim, time| {
		let cannon = &sim.cannon;
		println!(
			"t={:.3} |   pos=({:7.3}, {:7.3})   vel=({:7.3}, {:7.3})",
			time.t, cannon.pos.x, cannon.pos.y, cannon.vel.x, cannon.vel.y
		);
	});

	exec.add_job(Phase::Shutdown, |sim, time| {
		println!("==================================================");
		println!("Simulation completed at t={:.3}", time.t);
		println!(
			"Final cannonball position: ({:.3}, {:.3})",
			sim.cannon.pos.x, sim.cannon.pos.y
		);
		println!("==================================================");
	});

	exec.run(cannon_sim);

	println!("Simulation data saved to {filename}");
}
