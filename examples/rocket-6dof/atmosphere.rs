//! A basic atmosphere model.
//!
//! Source: [NASA Earth Atmosphere Model]
//!
//! [NASA Earth Atmosphere Model]: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html

/// Returns the pressure (kPa) at the given altitude and temperature.
pub fn get_pressure(altitude_m: f64, temperature_degc: f64) -> f64 {
	if altitude_m > 25_000.0 {
		return 2.488 * f64::powf((temperature_degc + 273.1) / 216.6, -11.388);
	}

	if altitude_m > 11_000.0 {
		return 22.65 * f64::exp(1.73 * (1.57e-4 * altitude_m));
	}

	101.29 * f64::powf((temperature_degc + 273.1) / 288.08, 5.256)
}

/// Returns the temperature (°C) at the given altitude.
pub fn get_temperature(altitude_m: f64) -> f64 {
	if altitude_m > 25_000.0 {
		return -131.21 + (2.99e-3 * altitude_m);
	}

	if altitude_m > 11_000.0 {
		return -56.46;
	}

	15.04 - (0.00649 * altitude_m)
}

/// Returns the air density (kg/m^3) at the given altitude.
pub fn get_air_density(altitude_m: f64) -> f64 {
	let temperature = get_temperature(altitude_m);
	let pressure = get_pressure(altitude_m, temperature);

	pressure / (0.2869 * (temperature + 273.1))
}
