# Carla Utility
A list of utility functions for CARLA and beyond to allow for quicker prototyping.

## How to contribute
Do not directly push code changes beyond a few lines. Create a PR instead, we will review each others PRs.

## Recording

Spin up a CARLA server and run the following -

```bash
python3 -m carla_utils.record output.lmp
```

To visualize this recording,

```bash
python3 examples/visualize_recording output.lmp
```

Note: you will no longer need the server running!
