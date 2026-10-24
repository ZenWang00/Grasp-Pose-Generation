# Grasp Pose Generation — ROS2 integration

This repository continues the robot-side development of the Semantic Shared Control project. It owns device and ROS2 adapters, measured feedback, runtime coordination, final command acceptance, lifecycle and controller authority. Reusable algorithms, shared contracts, validation and replay tools are maintained in [semantic-shared-control](https://github.com/ZenWang00/semantic-shared-control).

The ROS2 code consumes a pinned version of the server-owned pure library. The library does not depend on this repository. Repository ownership is separate from deployment: fusion and optimization initially run on the robot computer, while the robot-side gate retains final motion authority. See [Repository Ownership](https://github.com/ZenWang00/semantic-shared-control/wiki/Repository-Ownership-and-Cross-Repository-Workflow) for the complete boundary.

The current design is proposed work. The existing adapters do not establish that the shared-control contracts, stop behavior or timing guarantees have been implemented or validated. Phase 1's synthetic controller and complete offline runner belong here; that phase requires no ROS graph, device drivers or hardware.

## Shared project documentation and work

- [Canonical Wiki](https://github.com/ZenWang00/semantic-shared-control/wiki) — project design and contribution policy; its reviewed source lives in [server docs/wiki](https://github.com/ZenWang00/semantic-shared-control/tree/main/docs/wiki).
- [Shared development Project](https://github.com/users/ZenWang00/projects/3) — tasks from both repositories, with one global WIP limit and the Repository field identifying ownership.
- [Phase 1 Epic](https://github.com/ZenWang00/semantic-shared-control/issues/2) — the authoritative child checklist and aggregate acceptance.
- [Development Workflow](https://github.com/ZenWang00/semantic-shared-control/wiki/Development-Workflow), [Issue Standards](https://github.com/ZenWang00/semantic-shared-control/wiki/Issue-Standards), and [Project Board](https://github.com/ZenWang00/semantic-shared-control/wiki/Project-Board) — read before contributing. Add new ROS2 project Issues explicitly to Project #3 and verify their fields.

Keep device-specific setup, launch and adapter documentation with this code. Link shared contracts and policy instead of maintaining a second Wiki copy. Cross-repository PRs and integration evidence identify both exact source revisions and the tested dependency/configuration versions.

## Preserved source baseline

The annotated Git tag [v1.0.0](https://github.com/ZenWang00/Grasp-Pose-Generation/tree/v1.0.0) points to commit `00a42b32ef6d82863d298ccd876d0ee9523afa5d` (tag object `06eec60d8682d18b58d5e8a56d12a38e05afa962`). No GitHub Release was published when checked on 24 October 2026. Continue development through task branches and reviewed PRs to `main`, preserving this tag. It is a source baseline, not a validated shared-control release or tested server/ROS2 version pair.
