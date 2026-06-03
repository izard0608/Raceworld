# Project Rules

This repository is a ROS workspace for developing a visual line-following feature.

Development constraints:

- Use `follow_new.py` as the only implementation file for the visual line-following feature.
- Do not modify ROS interfaces, including topics, message types, service/action definitions, node interface contracts, launch-facing names, or package-level ROS wiring.
- Treat `env.txt` as the fixed Python environment reference.
- Do not change the Python environment, install new dependencies, or require packages that are not available in `env.txt`.
- Before implementing changes, verify that the intended code path fits the existing ROS interfaces and the Python environment described by `env.txt`.
- If a requested change cannot be completed within `follow_new.py` without changing ROS interfaces or the Python environment, report the limitation instead of modifying other project files.
