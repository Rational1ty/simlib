use std::collections::HashMap;

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
	Integrate,
	PostIntegrate,
	Shutdown,
}

type Job<S> = Box<dyn FnMut(&mut S, &SimTime)>;

pub struct Executor<S> {
	time: SimTime,
	end_time: f64,
	jobs: HashMap<Phase, Vec<Job<S>>>,
}

impl<S> Executor<S> {
	pub fn new(dt: f64, end_time: f64) -> Self {
		Self {
			time: SimTime { t: 0.0, dt, step: 0 },
			end_time,
			jobs: HashMap::new(),
		}
	}

	pub fn add_job<F>(&mut self, phase: Phase, job: F)
	where
		F: FnMut(&mut S, &SimTime) + 'static,
	{
		self.jobs
			.entry(phase)
			.or_default()
			.push(Box::new(job));
	}

	pub fn run(&mut self, state: &mut S) {
		self.run_phase(Phase::Init, state);

		while self.time.t < self.end_time {
			self.run_phase(Phase::PreIntegrate, state);
			self.run_phase(Phase::Integrate, state);
			self.run_phase(Phase::PostIntegrate, state);

			self.time.step += 1;
			self.time.t = self.time.dt * self.time.step as f64;
		}

		self.run_phase(Phase::Shutdown, state);
	}

	fn run_phase(&mut self, phase: Phase, state: &mut S) {
		if let Some(jobs) = self.jobs.get_mut(&phase) {
			for job in jobs {
				job(state, &self.time);
			}
		}
	}
}
