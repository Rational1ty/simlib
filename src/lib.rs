#![expect(
	clippy::type_complexity,
	reason = "The types this lint catches aren't complex enough to warrant their own aliases."
)]

mod dynamic_event;
mod executor;
mod integrator;
mod recorder;

pub use dynamic_event::CrossingMode;
pub use executor::{Executor, Phase, SimTime};
pub use integrator::runge_kutta_4;
pub use recorder::Recorder;
