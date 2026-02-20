// TODO: remove this once sim is finished
#![allow(dead_code)]

use std::f64::consts::PI;

use glam::dvec2;
use simlib::{Executor, Phase, Recorder};

use crate::{aero::BodyAeroCoefficients, lut::Lut1, motor::Motor, rocket::Rocket};

mod aero;
mod atmosphere;
mod lut;
mod motor;
mod rocket;

fn main() {
	// this is modeled roughly after an IRIS 4 rocket with an H/I motor
	let sim = Rocket {
		coeffs: BodyAeroCoefficients {
			cp: 1.625,
			cg: 1.466,
			surface_area: 8.13e-3,
			ca_mach: Lut1::new(
				&[0.0, 0.025, 0.12, 0.4, 1.0, 1.1, 1.27, 3.0],
				&[1.95, 0.85, 0.73, 0.75, 0.91, 0.96, 0.84, 0.62],
			),
			cn_alpha_mach: Lut1::new(
				&[0.0, 0.4, 0.6, 1.0, 1.2, 1.32, 1.4, 2.0, 3.0],
				&[21.0, 21.27, 21.66, 23.12, 23.84, 23.48, 22.72, 14.22, 9.5],
			),
		},
		inertia: 0.62,
		mass: 2.3,
		motor: Motor {
			thrust_curve: Lut1::new(
				&[0.0, 0.2, 1.8, 2.0, 20.0],
				&[500.0, 500.0, 500.0, 0.0, 0.0],
			),
		},
		..Default::default()
	};

	let dt = 0.1;
	let end_time = 20.0;
	let mut exec = Executor::<Rocket>::new(dt, end_time);

	exec.set_integrator(
		|sim, _| {
			vec![
				sim.position.x,
				sim.position.y,
				sim.velocity.x,
				sim.velocity.y,
				sim.orientation,
				sim.angular_vel,
			]
		},
		|sim, time| sim.derivative(time),
		|sim, s| {
			sim.position.x = s[0];
			sim.position.y = s[1];
			sim.velocity.x = s[2];
			sim.velocity.y = s[3];
			sim.orientation = s[4];
			sim.angular_vel = s[5];
		},
	);

	exec.add_job(Phase::Init, |sim, _| {
		// simulate launching from a rail
		sim.position = dvec2(0.0, 3.0);
		sim.velocity = dvec2(0.0, 25.0);
		sim.orientation = PI / 2.0; // 90 degrees is vertical
		sim.angular_vel = 0.0;

		println!("Starting sim with initial state: {:#?}", sim);
	});

	exec.add_job(Phase::PreIntegrate, |sim, time| {
		let Rocket {
			position,
			velocity,
			orientation,
			angular_vel,
			..
		} = &sim;
		println!(
			"t={:.2} | pos=({:.3}, {:.3}) vel=({:.3}, {:.3}) orientation={:.3} angular_vel={:.3}",
			time.t, position.x, position.y, velocity.x, velocity.y, orientation, angular_vel
		);
	});

	exec.add_job(Phase::PostIntegrate, |sim, time| {
		if sim.position.y < 0.0 {
			eprintln!("Underground! t = {}", time.t);
		}
		if sim.velocity.y < 0.0 {
			eprintln!("Falling down! t = {}", time.t);
		}
	});

	let mut recorder = Recorder::<Rocket>::new("rocket-3dof.csv");
	recorder.track("pos_x", |sim| sim.position.x);
	recorder.track("pos_y", |sim| sim.position.y);
	recorder.track("vel_x", |sim| sim.velocity.x);
	recorder.track("vel_y", |sim| sim.velocity.y);
	recorder.track("acc_x", |sim| sim.acceleration.x);
	recorder.track("acc_y", |sim| sim.acceleration.y);
	recorder.track("orientation", |sim| sim.orientation);
	recorder.track("angular_vel", |sim| sim.angular_vel);
	recorder.track("angular_accel", |sim| sim.angular_accel);

	exec.set_recorder(recorder);

	exec.run(sim);
}
