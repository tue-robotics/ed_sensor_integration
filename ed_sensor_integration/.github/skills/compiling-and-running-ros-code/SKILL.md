---
name: compiling-and-running-ros-code
description: How to compile and build ROS C++ packages in this workspace. Use this skill whenever you need to build, compile, or validate ROS code changes — including after editing C++ source files, fixing build errors, or verifying that code compiles successfully.
---

# Compiling and Running ROS Code

## Building packages

Use `tue-make` to build ROS packages:

```bash
# Build a specific package
tue-make <package_name>

# Example:
tue-make ed_sensor_integration

# Build everything (all packages)
tue-make
```

## When to build

- After editing C++ source files, always run `tue-make <package_name>` to validate the change compiles.
- When build errors need to be resolved, run `tue-make <package_name>` after each fix to confirm.
- Use `tue-make` (no arguments) to rebuild the entire workspace when cross-package changes are made.
