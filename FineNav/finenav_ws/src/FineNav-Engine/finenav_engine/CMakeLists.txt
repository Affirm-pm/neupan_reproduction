cmake_minimum_required(VERSION 3.22)
project(finenav_engine)

find_package(ament_cmake REQUIRED)

# Meta-package: no targets of its own.  It exists so `apt install finenav_engine`
# (and a future finenav_engine_py) resolve together, and so the C++ engine's
# package name stays free to change without touching users.
ament_package()
