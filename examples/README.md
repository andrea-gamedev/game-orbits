# Examples

Examples are rendered using the Bevy engine, which is not included by default to keep the library lightweight. In order to run examples, you must include the `example` feature flag which adds the Bevy engine as well as any features needed from the Bevy engine to render the examples[\*](#ref-1)</sup>.

```
cargo run --example <example_name> --features example
```

Replace `<example_name>` with the name of the example you wish to run in order to run it with Bevy.

<sup id="ref-1">\* This crate does have a feature flag `bevy`, but this only adds the bare minimum to be able to use the `PlanetDatabase` in the Bevy ECS, and does not include any of the features needed by the examples specifically, like rendering or input handling</sup>

## List of Examples

example name      | description
------------------|-------------
`low_earth_orbit` | Renders a camera orbiting the earth at about the altitude of the international space station. Tests that the math used in the library can render actual visible orbital motion smoothly without skips or jitters.
`solar_system`    | Interactive model of the solar system testing things like nested orbits and direction calculation