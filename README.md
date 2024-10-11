# Template for Isaac Lab Projects

[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.0.0-silver.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-1.0.0-silver)](https://isaac-sim.github.io/IsaacLab)
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Windows platform](https://img.shields.io/badge/platform-windows--64-orange.svg)](https://www.microsoft.com/en-us/)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit&logoColor=white)](https://pre-commit.com/)
[![License](https://img.shields.io/badge/license-MIT-yellow.svg)](https://opensource.org/license/mit)

## Overview

This repository serves as a template for building projects or extensions based on Isaac Lab. It allows you to develop in
an isolated environment, outside of the core Isaac Lab repository.

**Key Features:**

- `Isolation` Work outside the core Isaac Lab repository, ensuring that your development efforts remain self-contained.
- `Flexibility` This template is set up to allow your code to be run as an extension in Omniverse.

**Keywords:** extension, template, isaaclab

### Installation

- Install Isaac Lab, see
  the [installation guide](https://isaac-sim.github.io/IsaacLab/source/setup/installation/index.html). **Please use
  [IsaacLab v1.0.0 with IsaacSim 4.0.0](https://github.com/isaac-sim/IsaacLab/blob/3ad18a8e1a5c166ad1a22f105d47a5c578de68d7/docs/source/setup/installation/pip_installation.rst)**.

- Using a python interpreter that has Isaac sLab installed, install the library

```
cd exts/berkeley_humanoid
python -m pip install -e .
```

## Run

Training an agent with RSL-RL on Velocity-Rough-Berkeley-Humanoid-v0:

```
# run script for training
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/rsl_rl/train.py --task Velocity-Rough-Berkeley-Humanoid-v0
# run script for playing
${ISAAC_LAB_PATH}/isaaclab.sh -p scripts/rsl_rl/play.py --task Velocity-Rough-Berkeley-Humanoid-Play-v0
```

## FAQ
**Q: Why doesn't the maximum torque of each joint match the values in the paper?**

**A:** The maximum torque is limited for safety reasons.

**Q: Where is the joint armature from?**

**A:** From CAD system.

**Q: Why does the friction of each joint so large?**

**A:** The motor we used has large cogging torque, we include it in the friction of the actuator model.
