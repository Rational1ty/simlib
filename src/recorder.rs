use std::fs::File;
use std::io::{self, Seek, Write};
use std::path::Path;

pub struct Recorder<S> {
	names: Vec<String>,
	accessors: Vec<Box<dyn Fn(&S) -> f64>>,
	times: Vec<f64>,
	data: Vec<Vec<f64>>,
	file: File,
}

impl<S> Recorder<S> {
	pub fn new<P: AsRef<Path>>(file_path: P) -> Self {
		let file = File::create(file_path).unwrap();
		Self {
			names: Vec::new(),
			accessors: Vec::new(),
			times: Vec::new(),
			data: Vec::new(),
			file,
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

	pub(crate) fn sample_and_write(&mut self, state: &S, t: f64) {
		self.sample(state, t);
		let row = self.data.last().unwrap();
		let data_str = row
			.iter()
			.map(|x| x.to_string())
			.collect::<Vec<_>>()
			.join(",");
		let pos = self.file.stream_position().unwrap();
		if pos == 0 {
			eprintln!("Writing CSV header");
			write!(self.file, "time,").unwrap();
			writeln!(self.file, "{}", self.names.join(",")).unwrap();
		}
		writeln!(self.file, "{t},{data_str}").unwrap();
	}

	pub(crate) fn write_csv(&mut self) -> io::Result<()> {
		assert!(self.times.len() == self.data.len());
		let pos = self.file.stream_position().unwrap();
		if pos != 0 {
			eprintln!("CSV file already written; will not write again");
			return Ok(());
		}

		// let mut file = File::create(&self.file_path)?;

		// header
		write!(self.file, "time,")?;
		writeln!(self.file, "{}", self.names.join(","))?;

		// data rows
		for (t, data) in self.times.iter().zip(&self.data) {
			let data_str = data
				.iter()
				.map(|x| x.to_string())
				.collect::<Vec<_>>()
				.join(",");
			writeln!(self.file, "{t},{data_str}")?;
		}

		Ok(())
	}
}
