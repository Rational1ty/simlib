// TODO: remove this once sim is finished
#![allow(dead_code)]

use simlib::{Executor, Phase, Recorder};

use crate::{
	aero::BodyAeroCoefficients,
	lut::Lut1,
	motor::Motor,
	rocket::{FlightPhase, Rail, Rocket, velocity_to_mach},
};

mod aero;
mod atmosphere;
mod lut;
mod motor;
mod rocket;

fn main() {
	let motor = Motor::from_eng_file("I280.eng").unwrap();
	let rail = Rail {
		angle: 85_f64.to_radians(),
		length: 3.084, // 10 feet
	};

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
		mass: 2.0,
		motor,
		rail,
		..Default::default()
	};

	let dt = 0.01;
	let end_time = 35.0;
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
		// println!("Starting sim with initial state: {:#?}", sim);
		sim.orientation = sim.rail.angle;
		println!("Starting rocket sim");
		println!("  motor: {}", sim.motor.designation);
		println!("  rail: {} m at {:.2}°", sim.rail.length, 90.0 - sim.rail.angle.to_degrees());
	});

	exec.add_job(Phase::PreIntegrate, |sim, time| {
		let steps_per_sec = (1.0 / time.dt).round() as u64;
		if time.step % steps_per_sec != 0 {
			return;
		}

		let Rocket {
			position,
			velocity,
			orientation,
			angular_vel,
			..
		} = &sim;
		println!(
			"t={:.3} | pos=({:.3}, {:.3}) vel=({:.3}, {:.3}) orientation={:.3} angular_vel={:.3}",
			time.t, position.x, position.y, velocity.x, velocity.y, orientation, angular_vel
		);
	});

	let mut prev_phase = FlightPhase::OnRail;
	exec.add_job(Phase::PostIntegrate, move |sim, time| {
		if sim.flight_phase == FlightPhase::Boost && time.t > sim.motor.burn_time_end {
			sim.flight_phase = FlightPhase::Coast;
		}
		if sim.flight_phase == FlightPhase::Coast && sim.velocity.y < 0.0 {
			sim.flight_phase = FlightPhase::Descent;
		}
		if sim.flight_phase == FlightPhase::Descent && sim.position.y <= 0.0 {
			sim.flight_phase = FlightPhase::Ground;
		}

		if sim.flight_phase != prev_phase {
			println!("Phase {:?}, t = {}", sim.flight_phase, time.t);
			prev_phase = sim.flight_phase;
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
	recorder.track("mach", |sim| velocity_to_mach(sim.velocity.length(), sim.position.y));

	exec.set_recorder(recorder);

	exec.run(sim);
}
