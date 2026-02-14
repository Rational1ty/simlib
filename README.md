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
- [ ] State integration (RK4 and other integrators)
- [x] Variable recording
- [ ] Last step checkpointing
- [ ] Dynamic events (regula falsi method)
- [ ] Monte carlo (with `rayon` crate)

Other features:
- [ ] Run jobs at different rates
- [ ] Input files / scripting
- [ ] Variable server
- [ ] Arbitrary checkpointing (any point in the sim)
- [ ] Real-time simulation
- [ ] Freeze/unfreeze
- [ ] Proc macro for variable recording
- [ ] Multi-rate integration
- [ ] Built-in variable units (maybe)

## Design

Simulation executor is generic over `S`, the type of the simulation:

```rust
struct RocketSimulation {
	position: Vec3,
	velocity: Vec3,
	acceleration: Vec3,
	orientation: Quat,
	fake_sensor: f32,
}

let exec = Executor::<RocketSimulation>::new();
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
	run integration loop
	run post-integration jobs

	for each dynamic_event job:
		run error function
		if value changed sign:
			rewind to previous time step
			integrate to t_event
			run event job
	
	record variables

	step += 1
	t = step * dt

run shutdown jobs
save recorded variables to file
```


## Need to figure out

- How to handle multiple dynamic events firing within the same time step
- If the current design scales well to larger and more complex sims

