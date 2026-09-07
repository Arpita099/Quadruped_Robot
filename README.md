# Sim-to-Sim Quadruped Locomotion in MuJoCo

This is a small robotics project where I run a **pretrained reinforcement learning locomotion policy** for a **Unitree Go2 quadruped robot** in MuJoCo.

I did not train the RL policy from scratch. The pretrained policy is already provided as `policy.pt`. My main work in this project was to understand how the policy works with the robot state, how the observation is prepared, how the joint order is converted, and how the policy output is finally used to control the robot in MuJoCo.

## Project Flow

The basic flow of the project is:

```text
MuJoCo robot state
        ↓
Build observation
        ↓
Convert joint/state order
        ↓
Pretrained RL policy
        ↓
12 joint actions
        ↓
Convert actions to target joint positions
        ↓
PD controller
        ↓
MuJoCo robot
```

The policy takes the current state of the robot and predicts the action for the 12 joints.

## Observation

The policy uses a **48-dimensional observation**.

It contains:

| Observation           | Dimension |
| --------------------- | --------: |
| Base linear velocity  |         3 |
| Base angular velocity |         3 |
| Projected gravity     |         3 |
| Motion command        |         3 |
| Joint position        |        12 |
| Joint velocity        |        12 |
| Previous action       |        12 |
| Total                 |        48 |

So before giving anything to the policy, I need to collect these values from MuJoCo and keep them in the same format that the policy expects.

## Policy Output

The policy gives **12 actions** because the Go2 robot has 12 controlled joints.

The action is converted to a target joint position using:

```python
target_dof_pos = action * action_scale + default_angles
```

After that, the target joint positions are sent to the low-level controller.

## Joint Order Conversion

One important part of this project is the joint ordering.

The pretrained policy and MuJoCo do not use the joint values in exactly the same order, so I use two functions:

```python
isaac2mujoco(inputs)
mujoco2isaac(inputs)
```

These functions convert the joint values between the policy format and MuJoCo format.

If the joint order is wrong, the action can be sent to the wrong leg joint and the robot movement will not be correct.

## PD Control

The policy does not directly give the final torque.

First, the policy action is converted to a target joint position. Then the PD controller calculates the torque from the joint position error and joint velocity.

The simple idea is:

```text
target joint position
        ↓
compare with current joint position
        ↓
PD controller
        ↓
joint torque
        ↓
robot movement
```

Torque limits are also used so that the applied torque stays inside the allowed range.

## Simulation and Control Frequency

The simulation timestep is:

```python
simulation_dt = 0.002
```

So MuJoCo runs at around **500 Hz**.

The control decimation is:

```python
control_decimation = 10
```

That means the policy is not called at every physics step. It is called after every 10 simulation steps.

So the policy runs at around **50 Hz**.

## Default Command

The current command is:

```python
cmd_init = [1.5, 0.0, 0.0]
```

This command is also included in the observation and tells the policy what type of motion is expected.

## Repository Structure

```text
.
├── sim2sim.py
├── policy.pt
└── robots/
    ├── go1/
    └── go2/
```

For this project I am using:

```text
robots/go2/scene.xml
```

## Installation

Required packages:

```bash
pip install mujoco numpy torch scipy
```

Then run:

```bash
python sim2sim.py
```

The MuJoCo viewer will open and the Go2 robot will run using the pretrained locomotion policy.

## Main Configuration

Some important settings are:

```python
config = {
    "policy_path": "./policy.pt",
    "xml_path": "./robots/go2/scene.xml",
    "simulation_duration": 3000,
    "simulation_dt": 0.002,
    "control_decimation": 10,
    "kps": 50.0,
    "kds": 0.1,
    "action_scale": 0.2,
    "num_actions": 12,
    "num_obs": 48,
    "cmd_init": [1.5, 0.0, 0.0]
}
```

One thing I found from the current code is that the `kps` and `kds` values from the config are not fully used inside the final torque calculation because `joint_torque()` has its own default values.

This part can be improved later.

## What I Learned

From this project I mainly learned how a pretrained RL policy is connected with a robot simulator.

I also learned about:

* robot state representation
* observation and action space
* joint ordering
* quadruped locomotion
* policy inference
* PD control
* torque control
* control frequency
* sim-to-sim policy transfer

This project also helped me understand that using a trained RL policy on another simulator is not only about loading the model. The robot state, joint order, action format, controller, and simulation settings also need to match correctly.

## Future Work

In the future I want to test the policy under different conditions, for example:

* different ground friction
* different robot mass
* sensor noise
* observation delay
* external disturbance
* different velocity commands

I am interested in seeing how these changes affect the stability of the robot and how robust the pretrained policy is.

## Note

The RL policy used in this repository is pretrained. The training code is not included here.

My work in this project is mainly focused on running the policy in MuJoCo, understanding the robot-policy interface, and studying the control pipeline.
