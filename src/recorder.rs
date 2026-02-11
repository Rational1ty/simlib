use std::fs::File;
use std::io::Write;
use std::path::Path;

pub struct Recorder<S> {
	names: Vec<String>,
	accessors: Vec<Box<dyn Fn(&S) -> f64>>,
	times: Vec<f64>,
	data: Vec<Vec<f64>>,
}

impl<S> Recorder<S> {
	pub fn new() -> Self {
		Self {
			names: Vec::new(),
			accessors: Vec::new(),
			times: Vec::new(),
			data: Vec::new(),
		}
	}

	pub fn track<F>(&mut self, name: &str, accessor: F)
	where
		F: (Fn(&S) -> f64) + 'static,
	{
		self.names.push(name.to_string());
		self.accessors.push(Box::new(accessor));
	}

	pub(crate) fn sample(&mut self, state: &S, t: f64) {
		self.times.push(t);
		let row: Vec<f64> = self.accessors.iter().map(|f| f(state)).collect();
		self.data.push(row);
	}

	pub(crate) fn write_csv<P: AsRef<Path>>(&self, path: P) -> std::io::Result<()> {
		assert!(self.times.len() == self.data.len());

		let mut file = File::create(path)?;

		// header
		write!(file, "time, ")?;
		writeln!(file, "{}", self.names.join(", "))?;

		// data rows
		for (t, data) in self.times.iter().zip(&self.data) {
			let data_str = data
				.iter()
				.map(|x| x.to_string())
				.collect::<Vec<_>>()
				.join(", ");
			writeln!(file, "{}, {}", t, data_str)?;
		}

		Ok(())
	}
}
