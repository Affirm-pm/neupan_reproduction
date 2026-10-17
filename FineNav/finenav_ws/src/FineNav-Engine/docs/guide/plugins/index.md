# FineNav Plugins

Find Map, Planner, and related components to use with FineNav-Engine.
Plugins may be included in the engine repository or distributed in a separate
integration repository.

## Browse plugins

| Category | What to look for |
| --- | --- |
| [Maps](maps.md) | Environment representations and the queries they provide. |
| [Planners](planners.md) | Planning algorithms, supported inputs, and navigation outputs. |

The catalogue currently documents the built-in example plugins.
Separate integration repositories will be added with their own installation
instructions and compatibility information.

## Use a plugin

1. Choose a Map and Planner that suit your robot and environment.
2. Follow the plugin repository's installation instructions.
3. Use a supplied WorldView or [write one](../worldview.md) for the selected pair.
4. Load the installed plugin libraries and select the WorldView in your
   [C++ application](../cpp-application.md).

The built-in example combines `dummy_map`, `dummy_planner`, and
`dummy_world_view` in the `finenav_example` package.
Follow the [Quickstart](../quickstart.md) to build and run that combination.