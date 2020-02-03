# yoshicar
Software for a self-driving robocar using ROS

![pic](docs/yoshicar.jpg)
   
# Notes

## ESC

The ESC is a little weird because it has two backword drive modes.
When using equal power, the car drives faster in backwards than in forwards.
This can be overcome by hitting neutral very quickly after going backwards and then going backwards again.
So it looks like:

backwards -> neutral -> backwards

Then the car drives considerably slower backwards.
