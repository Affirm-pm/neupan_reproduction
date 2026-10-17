# Offline solver dependencies

`src/` contains unmodified source snapshots of OSQP, QDLDL and osqp-eigen from
the pre-existing local dependency checkout. These complete source snapshots
are now bundled here; the original checkout is not needed.
Each source directory retains its upstream license. No other FineNav checkout
is required by the resulting build or runtime.

Rebuild with `finenav_example/create_custom_neupan_planner/tools/build.sh` in the
main workspace source tree. Generated `build/` and `install/` may be recreated.
