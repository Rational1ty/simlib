use std::collections::HashMap;

use crate::{
	dynamic_event::{CrossingMode, DynamicEvent, RegulaFalsi},
	integrator::{Integrator, runge_kutta_4},
	recorder::Recorder,
};

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

// TODO: add builder to validate configuration (e.g. dyn events with no integrator)
// before starting sim
pub struct Executor<S> {
	time: SimTime,
	end_time: f64,
	jobs: HashMap<Phase, Vec<Job<S>>>,
	dyn_events: Vec<RegulaFalsi<S>>,
	integrator: Option<Integrator<S>>,
	recorder: Option<Recorder<S>>,
}

impl<S> Executor<S> {
	pub fn new(dt: f64, end_time: f64) -> Self {
		Self {
			time: SimTime {
				t: 0.0,
				dt,
				step: 0,
			},
			end_time,
			jobs: HashMap::new(),
			dyn_events: Vec::new(),
			integrator: None,
			recorder: None,
		}
	}

	pub fn set_recorder(&mut self, recorder: Recorder<S>) {
		self.recorder = Some(recorder);
	}

	pub fn set_integrator<L, D, U>(&mut self, state_loader: L, derivative: D, state_unloader: U)
	where
		L: Fn(&mut S, &SimTime) -> Vec<f64> + 'static,
		D: Fn(&mut S, &SimTime) -> Vec<f64> + 'static,
		U: Fn(&mut S, &[f64]) + 'static,
	{
		self.integrator = Some(Integrator {
			state_loader: Box::new(state_loader),
			derivative: Box::new(derivative),
			state_unloader: Box::new(state_unloader),
		});
	}

	pub fn add_dynamic_event<E, F>(&mut self, error_fn: E, mode: CrossingMode, action: F)
	where
		E: Fn(&S) -> f64 + 'static,
		F: FnMut(&mut S, &SimTime) + 'static,
	{
		let rf = RegulaFalsi::new(mode, error_fn, action);
		self.dyn_events.push(rf);
	}

	pub fn add_job<F>(&mut self, phase: Phase, job: F)
	where
		F: FnMut(&mut S, &SimTime) + 'static,
	{
		self.jobs.entry(phase).or_default().push(Box::new(job));
	}

	pub fn run(&mut self, mut sim: S) {
		self.run_phase(Phase::Init, &mut sim);

		while self.time.t < self.end_time {
			self.run_phase(Phase::PreIntegrate, &mut sim);
			// TODO: update dynamic events here to avoid having lower bound undefined at start?

			if let Some(integrator) = &mut self.integrator {
				let Integrator {
					state_loader,
					derivative,
					state_unloader,
				} = integrator;

				let t_from = self.time.t;
				let t_to = t_from + self.time.dt;

				runge_kutta_4(
					&mut sim,
					state_loader,
					derivative,
					state_unloader,
					self.time.dt,
					&SimTime {
						t: t_from,
						..self.time
					},
				);

				self.run_dynamic_events(&mut sim, t_to);
			}

			self.time.step += 1;
			self.time.t = self.time.dt * self.time.step as f64;

			self.run_phase(Phase::PostIntegrate, &mut sim);

			if let Some(recorder) = &mut self.recorder {
				recorder.sample(&sim, self.time.t);
			}
		}

		self.run_phase(Phase::Shutdown, &mut sim);

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

	/// Updates all dynamic events and runs any events that triggered during during the time step.
	///
	/// If any dynamic events occurred during the step, we need to process them in order of
	/// increasing time. To do this, we check the `time_to_go` of each event, and process them in
	/// increasing order. Techincally, this could result in events executing in the wrong order if
	///
	/// - multiple events happen during the time step, and
	/// - the initial tgo estimates are in the wrong order (i.e., event 1 happens before event 2, but the first
	/// `tgo` for event 1 is after the first `tgo` for event 2).
	///
	/// But this is very unlikely to ever happen, so we don't account for it.
	fn run_dynamic_events(&mut self, sim: &mut S, mut t_to: f64) {
		let Integrator {
			state_loader,
			derivative,
			state_unloader,
		} = self.integrator.as_ref().unwrap();

		let mut events_fired: Vec<_> = self
			.dyn_events
			.iter_mut()
			.filter_map(|ev| ev.time_to_go(&sim, t_to).map(|tgo| (ev, tgo)))
			.collect();
		events_fired.sort_by(|e1, e2| f64::total_cmp(&e1.1, &e2.1));

		let original_t_to = t_to;

		for (event, mut tgo) in events_fired {
			loop {
				if tgo == 0.0 {
					event.apply(
						sim,
						&SimTime {
							t: t_to,
							..self.time
						},
					);
					event.reset();

					if let Some(recorder) = &mut self.recorder {
						recorder.sample(&sim, t_to);
					}

					t_to = original_t_to;
					break;
				}

				let t_from = t_to;
				t_to += tgo;
				// println!("[DYN-EVENT] tgo={tgo} integrating from {t_from} to {t_to}, dt={tgo}");

				runge_kutta_4(
					sim,
					state_loader,
					derivative,
					state_unloader,
					tgo,
					&SimTime {
						t: t_from,
						..self.time
					},
				);

				tgo = event.time_to_go(&sim, t_to).unwrap();
			}
		}
	}
}
