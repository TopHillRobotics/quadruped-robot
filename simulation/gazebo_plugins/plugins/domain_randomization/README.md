## Domain Randomization plugin

This multi-purpose Domain Randomization plugin allows changing both physical and visual model properties, namely:

1. Change gravity 3D vector;
1. Scale spawned model;
1. Change individual link properties, such as:
    1. Link's mass;
    1. Link's inertia tensor;
1. Change individual joint properties, such as:
    1. Joint limits (angle, effort and velocity);
    1. Joint friction;
    1. Joint damping;
1. Change joint PID controller parameters and target;
1. Change individual link's visual colors;
1. Change individual collision surface properties.

We provide an [Interface class] for interacting with the plugin, and an [example] client which uses this interface.

<!-- Links -->

[Domain Randomization plugin]: plugins/domain_randomization
[documented]: http://web.tecnico.ulisboa.pt/joao.borrego/gap/classDRInterface.html
[Interface class]: /../../tree/dev/utils
[example]: /../../tree/dev/examples/dr_example
