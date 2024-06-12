# Real-World-Robotics
In this semester course, students were grouped in teams of 4/5 to design and control an anthropomorphic robotic hand. My team and I designed an ambidextrous robotic hand, characterised by the four degree of freedom thumb. Such an anatomy could prove useful for ambidextrous tasks by preventing the robotic arm it is attached to from having to reconfigure itself in space.

The robotic hand was controlled with both reinforcement learning and teleoperation. The reinforcement learning uses the PPO algorithm implemented by the open-source repository `rl_games` and IsaacGym as the simulator. This followed the work done in this [paper](https://arxiv.org/abs/2308.02453). We trained the policy on rotating a sphere about all axes.

<img src="https://github.com/MattJmt/Real-World-Robotics/blob/main/figures/rwr_RL.gif" width="250"/> <img src="https://github.com/MattJmt/Real-World-Robotics/blob/main/figures/IsaacGymRLRotatingSphere.gif" width="250"/>

The teleoperation of the hand was done using a Luxonis OAK-D Pro stereo camera and the "DepthAI Hand Tracker" deep learning algorithm to capture the positions of the joints of the user's hand. These were then mapped to the hand's joint angles to effectively move each joint and finger.

<img src="https://github.com/MattJmt/Real-World-Robotics/blob/main/figures/rwr_teleop_cube.gif" width="250"/> <img src="https://github.com/MattJmt/Real-World-Robotics/blob/main/figures/rwr_teleop_fidget.gif" width="250"/>

The highlights of the "Challenge Day", in which the hand performs reinforcement and teleoperation tasks, can be found [here](https://www.youtube.com/watch?v=-YM1Ik7tEGE&ab_channel=MatthiasJammot).
<img src="https://github.com/MattJmt/Real-World-Robotics/blob/main/figures/hand%20diagram.png" width="350"/>

More information on this project is available in the "RWR_report".
