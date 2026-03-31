# simlib

A Rust library for building simulations, inspired by NASA's Trick simulation framework. This project
is a work in progress and not yet ready for serious use, but contributions and ideas are welcome.

---

## Plans

Critical path:
- [x] Simulation representation (just a struct for now)
- [x] Domain-agnostic simulation runner/executor (generic over simulation types)
- [x] Simulation timekeeping
- [x] Job scheduling
- [x] State integration (RK4 and other integrators)
- [x] Variable recording
- [x] Dynamic events
- [ ] Monte carlo

Other features:
- [ ] Scheduled jobs (time-based, not based on sim phase)
- [ ] Multiple simobject instances like in Trick
- [ ] Input files / scripting
- [ ] Variable server
- [ ] Checkpointing
- [ ] Real-time simulation
- [ ] Freeze/unfreeze
- [ ] Proc macro for variable recording
- [ ] Multi-rate integration
- [ ] Built-in variable units (maybe)

## Design

The simulation executor is generic over `S`, the type of the simulation:

```rust
struct RocketSimulation {
	position: Vec3,
	velocity: Vec3,
	acceleration: Vec3,
	orientation: Quat,
	fake_sensor: f64,
}

let dt = 0.1;
let end_time = 20.0;
let exec = Executor::<RocketSimulation>::new(dt, end_time);
```

The simulation is defined by calling methods and setting up callbacks on the executor:

```rust
exec.add_job(Phase::PostIntegrate, |sim, time| {
	println!("sim state = {:?}", sim);
	sim.fake_sensor = 5.0 * time.t;
});

exec.run(sim);
```

Main sim loop:

```
run init jobs

while t < end_time:
	run pre-integration jobs
	record variables

	run integration loop:
		load state vector
		load state derivatives
		integrate
		unload state vector

	run dynamic events:
		while tgo != 0:
			integrate with dt=tgo (might go backwards!)
			re-evaluate event
		run event action
		record variables

	step += 1
	t = step * dt

	run post-integration jobs
	record variables

run shutdown jobs
save recorded variables to file
```


## Need to figure out

- If the current design scales well to larger and more complex sims
- How to make sure integrator and derivative function always use the same number of state variables
- Best way to set up integration/derivative jobs
