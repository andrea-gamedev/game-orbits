# Examples

Examples are rendered using the bevy engine, and as such must be run with the `bevy` feature flag enabled

```
cargo run --example <example_name> --features example
```

Replace `<example_name>` with the name of the example you wish to run in order to run it with Bevy.

## List of Examples

example name      | description
------------------|-------------
`low_earth_orbit` | Renders a camera orbiting the earth at about the altitude of the international space station. Tests that the math used in the library can render actual visible orbital motion smoothly without skips or jitters.
`solar_system`    | Interactive model of the solar system testing things like nested orbits and direction calculation