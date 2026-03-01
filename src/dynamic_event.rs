use crate::SimTime;

pub trait DynamicEvent<S> {
	fn time_to_go(&mut self, sim: &S, step_start: f64) -> f64;
	fn apply(&self, sim: &mut S, time: &SimTime);
}

pub enum CrossingMode {
	Any,
	Increasing,
	Decreasing,
}

pub struct RegulaFalsi {
	prev_t: Option<f64>,
	prev_err: Option<f64>,
	error_tol: f64,
	mode: CrossingMode,
}

// sim -> error
// error -> tgo
// tgo -> iterate
// tgo == 0 -> apply
impl RegulaFalsi {

}

impl<S> DynamicEvent<S> for RegulaFalsi {
	fn time_to_go(&mut self, sim: &S, step_start: f64) -> f64 {
		todo!()
	}

	fn apply(&self, sim: &mut S, time: &SimTime) {
		
	}
}
