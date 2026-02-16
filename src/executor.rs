use std::collections::HashMap;

use crate::{integrators::Integrator, recorder::Recorder, runge_kutta_4};

#[derive(Clone, Copy, Debug)]
pub struct SimTime {
	pub t: f64,
	pub dt: f64,
	pub step: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Phase {
	Init,
	PreIntegrate,
	PostIntegrate,
	Shutdown,
}

type Job<S> = Box<dyn FnMut(&mut S, &SimTime)>;

pub struct Executor<S> {
	time: SimTime,
	end_time: f64,
	jobs: HashMap<Phase, Vec<Job<S>>>,
	integrator: Option<Integrator<S>>,
	last_state: S,
	recorder: Option<Recorder<S>>,
}

impl<S: Clone + Default> Executor<S> {
	pub fn new(dt: f64, end_time: f64) -> Self {
		Self {
			time: SimTime {
				t: 0.0,
				dt,
				step: 0,
			},
			end_time,
			jobs: HashMap::new(),
			integrator: None,
			last_state: S::default(),
			recorder: None,
		}
	}

	pub fn with_recorder(dt: f64, end_time: f64, recorder: Recorder<S>) -> Self {
		Self {
			recorder: Some(recorder),
			..Self::new(dt, end_time)
		}
	}

	pub fn set_integrator<L, D, U>(&mut self, state_loader: L, derivative: D, state_unloader: U)
	where
		L: Fn(&S) -> Vec<f64> + 'static,
		D: Fn(&[f64]) -> Vec<f64> + 'static,
		U: FnMut(&mut S, &[f64]) + 'static,
	{
		self.integrator = Some(Integrator {
			state_loader: Box::new(state_loader),
			derivative: Box::new(derivative),
			state_unloader: Box::new(state_unloader),
		});
	}

	pub fn add_job<F>(&mut self, phase: Phase, job: F)
	where
		F: FnMut(&mut S, &SimTime) + 'static,
	{
		self.jobs.entry(phase).or_default().push(Box::new(job));
	}

	pub fn run(&mut self, sim: &mut S) {
		self.run_phase(Phase::Init, sim);
		self.last_state = sim.clone();

		while self.time.t < self.end_time {
			self.run_phase(Phase::PreIntegrate, sim);

			if let Some(integrator) = &mut self.integrator {
				let Integrator {
					state_loader,
					derivative,
					state_unloader,
				} = integrator;

				let state = state_loader(sim);
				let integ_result = runge_kutta_4(&state, derivative.as_ref(), self.time.dt);
				state_unloader(sim, &integ_result);
			}

			self.run_phase(Phase::PostIntegrate, sim);

			self.time.step += 1;
			self.time.t = self.time.dt * self.time.step as f64;

			// checkpoint
			self.last_state = sim.clone();

			if let Some(recorder) = &mut self.recorder {
				recorder.sample(sim, self.time.t);
			}
		}

		self.run_phase(Phase::Shutdown, sim);

		if let Some(recorder) = &self.recorder {
			recorder.write_csv().unwrap();
		}
	}

	fn run_phase(&mut self, phase: Phase, sim: &mut S) {
		if let Some(jobs) = self.jobs.get_mut(&phase) {
			for job in jobs {
				job(sim, &self.time);
			}
		}
	}
}
