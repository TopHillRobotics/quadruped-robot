## Examples

This set of plugins was originally conceived so we could ultimately build the [scene generation] example.
It uses `CameraUtils`, `VisualUtils` and `WorldUtils` plugins.

Recently we also created an [example client] for our multi-purpose Domain Randomization plugin `DRPlugin`, in which we alter physical and visual properties of [Shadow's Dexterous Hand] during simulation.

In order to interact with our tools you need only write your own client applications that use our custom message definitions.
These are compiled into a shared library for your convenience, and can be easily linked to by installing our package.
See [external_example] for more details.

Furthermore, we provide simples example client applications to interact with each plugin.

- [camera_example], for acquiring frames using CameraUtils

- [visual_example], for changing object visuals using VisualUtils

- world_example was **deprecated** as most of its object generation utilities will no longer be supported. 

[Shadow's Dexterous Hand]: https://www.shadowrobot.com/products/dexterous-hand/
[example client]: dr_example
[external_example]: external_example
[scene generation]: scene_example
[camera_example]: camera_example 
[visual_example]: visual_example
