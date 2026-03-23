// TODO: remove this once sim is finished
#![expect(dead_code)]

use glam::dvec3;
use simlib::{Executor, Phase, Recorder};

use crate::{
	aero::BodyAeroCoefficients,
	lut::Lut2,
	motor::Motor,
	rocket::{FlightPhase, Rail, Rocket, velocity_to_mach},
};

mod aero;
mod atmosphere;
mod lut;
mod motor;
mod rocket;

fn main() {
	let sim = get_iris4_config();

	let dt = 0.01;
	let end_time = 30.0;
	let mut exec = Executor::<Rocket>::new(dt, end_time);

	exec.set_integrator(
		|sim, _| {
			vec![
				sim.position.x,
				sim.position.y,
				sim.position.z,
				sim.velocity.x,
				sim.velocity.y,
				sim.velocity.z,
				sim.orientation.x,
				sim.orientation.y,
				sim.orientation.z,
				sim.orientation.w,
				sim.angular_vel.x,
				sim.angular_vel.y,
				sim.angular_vel.z,
			]
		},
		|sim, time| sim.derivative(time),
		|sim, s| {
			sim.position.x = s[0];
			sim.position.y = s[1];
			sim.position.z = s[2];
			sim.velocity.x = s[3];
			sim.velocity.y = s[4];
			sim.velocity.z = s[5];
			sim.orientation = glam::DQuat::from_xyzw(s[6], s[7], s[8], s[9]).normalize();
			sim.angular_vel.x = s[10];
			sim.angular_vel.y = s[11];
			sim.angular_vel.z = s[12];
		},
	);

	exec.add_job(Phase::Init, |sim, _| {
		// println!("Starting sim with initial state: {:#?}", sim);
		sim.orientation = sim.rail.orientation();
		println!("Starting rocket sim");
		println!("  motor: {}", sim.motor.designation);
		println!(
			"  rail: {} m at elev {:.2}°, az {:.2}°",
			sim.rail.length,
			90.0 - sim.rail.angle.to_degrees(),
			sim.rail.azimuth.to_degrees(),
		);
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
			"t={:.3} | pos=({:.3}, {:.3}, {:.3}) vel=({:.3}, {:.3}, {:.3}) q=({:.4}, {:.4}, {:.4}, {:.4}) w=({:.4}, {:.4}, {:.4})",
			time.t,
			position.x,
			position.y,
			position.z,
			velocity.x,
			velocity.y,
			velocity.z,
			orientation.x,
			orientation.y,
			orientation.z,
			orientation.w,
			angular_vel.x,
			angular_vel.y,
			angular_vel.z,
		);
	});

	let mut prev_phase = FlightPhase::OnRail;
	exec.add_job(Phase::PostIntegrate, move |sim, time| {
		if sim.flight_phase == FlightPhase::Boost && time.t > sim.motor.burn_time_end {
			sim.flight_phase = FlightPhase::Coast;
		}
		if sim.flight_phase == FlightPhase::Coast && sim.velocity.z < 0.0 {
			sim.flight_phase = FlightPhase::Descent;
		}
		if sim.flight_phase == FlightPhase::Descent && sim.position.z <= 0.0 {
			sim.flight_phase = FlightPhase::Ground;
		}

		if sim.flight_phase != prev_phase {
			println!("Phase {:?}, t = {}", sim.flight_phase, time.t);
			prev_phase = sim.flight_phase;
		}
	});

	let mut recorder = Recorder::<Rocket>::new("rocket-6dof.csv");
	recorder.track("pos_x", |sim| sim.position.x);
	recorder.track("pos_y", |sim| sim.position.y);
	recorder.track("pos_z", |sim| sim.position.z);
	recorder.track("vel_x", |sim| sim.velocity.x);
	recorder.track("vel_y", |sim| sim.velocity.y);
	recorder.track("vel_z", |sim| sim.velocity.z);
	recorder.track("acc_x", |sim| sim.acceleration.x);
	recorder.track("acc_y", |sim| sim.acceleration.y);
	recorder.track("acc_z", |sim| sim.acceleration.z);
	recorder.track("q_x", |sim| sim.orientation.x);
	recorder.track("q_y", |sim| sim.orientation.y);
	recorder.track("q_z", |sim| sim.orientation.z);
	recorder.track("q_w", |sim| sim.orientation.w);
	recorder.track("omega_x", |sim| sim.angular_vel.x);
	recorder.track("omega_y", |sim| sim.angular_vel.y);
	recorder.track("omega_z", |sim| sim.angular_vel.z);
	recorder.track("alpha_x", |sim| sim.angular_accel.x);
	recorder.track("alpha_y", |sim| sim.angular_accel.y);
	recorder.track("alpha_z", |sim| sim.angular_accel.z);
	recorder.track("mach", |sim| velocity_to_mach(sim.velocity.length(), sim.position.z));

	// sim health metrics
	recorder.track("alpha", |sim| {
		let vel_body = sim.orientation.conjugate() * sim.velocity;
		f64::atan2(-vel_body.z, vel_body.x).to_degrees()
	});
	recorder.track("beta", |sim| {
		let vel_body = sim.orientation.conjugate() * sim.velocity;
		f64::atan2(vel_body.y, f64::hypot(vel_body.x, vel_body.z)).to_degrees()
	});

	exec.set_recorder(recorder);

	exec.run(sim);
}

fn get_iris4_config() -> Rocket {
	let iris_config = BodyAeroCoefficients {
		cp: 1.625,
		cg: 1.466,
		surface_area: 8.13e-3,
		..Default::default()
	};

	let iris_coeffs = read_hdf5("iris4.hdf5", iris_config).unwrap();

	let rail = Rail {
		angle: 85_f64.to_radians(),
		azimuth: 30_f64.to_radians(),
		length: 3.084, // 10 feet
	};
	let motor = Motor::from_eng_file("I280.eng").unwrap();

	Rocket {
		coeffs: iris_coeffs,
		mass: 2.0,
		inertia: dvec3(1.34e-2, 0.61, 0.61),
		rail,
		motor,
		..Default::default()
	}
}

fn get_telemachus_config() -> Rocket {
	let tele_coeffs = BodyAeroCoefficients {
		cp: 2.425,
		cg: 1.851,
		surface_area: 18.258e-3,
		..Default::default()
	};

	let tele_coeffs = read_hdf5("tele.hdf5", tele_coeffs).unwrap();

	let rail = Rail {
		angle: 85_f64.to_radians(),
		azimuth: 30_f64.to_radians(),
		length: 5.0,
	};

	let motor = Motor::from_eng_file("O3400.eng").unwrap();

	Rocket {
		coeffs: tele_coeffs,
		mass: 18.991,
		inertia: dvec3(0.061, 18.4, 18.4),
		rail,
		motor,
		..Default::default()
	}
}

fn read_hdf5(filename: &str, config: BodyAeroCoefficients) -> hdf5::Result<BodyAeroCoefficients> {
	let file = hdf5::File::open(filename)?;

	let angle: Vec<f64> = file.dataset("alpha")?.read_raw()?;
	let mach: Vec<f64> = file.dataset("mach")?.read_raw()?;
	let cx: Vec<f64> = file.dataset("ca/off")?.read_raw()?;
	let cy: Vec<f64> = file.dataset("cy/off")?.read_raw()?;
	let cz: Vec<f64> = file.dataset("cn/off")?.read_raw()?;

	eprintln!("angle = {:?}", angle);
	eprintln!("mach = {:?} .. {:?}", &mach[0..5], &mach[mach.len() - 5..]);

	Ok(BodyAeroCoefficients {
		cx_alpha_mach: Lut2::new(&angle, &mach, &cx),
		cy_beta_mach: Lut2::new(&angle, &mach, &cy),
		cz_alpha_mach: Lut2::new(&angle, &mach, &cz),
		..config
	})
}
