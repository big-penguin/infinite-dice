# infinite-dice
a robot that rolls a die endlessly. Future plans include "chasing pi" (roll the die over and over while looking for the longest consecutive sequence of pi digits - in theory up to 3.1415, due to the 6-side limit). An option to roll on-demand instead may be added as well. (if/when I feel like it...)

# what it does
Python code inclues a (very) crude IK solver for the EEZYbotARM mk1, openCV code for tracking the die and counting the dots (dot count isn't used as of this commit), as well as the bits required to actually move the robot and talk to the arduino. 

Arduino side is trivial, just listens for 4 bytes over serial (one byte per servo) and sets the servos to those angles. 

Also included is a gripper model, because the one included with the robot was both too small and too complicated for my tastes
