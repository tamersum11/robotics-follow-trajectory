# Robotics 101: Trajectory Following
In this study, has been simulated to trajectory following of differential drive robot in MATLAB SIMULINK and has been implemented to Python code.

## Simulation of Trajectory Following in MATLAB SIMULINK
### MATLAB SIMULINK Structure of Trajectory Following System
</br > ![Trajectory Following System](/png/hw321sml.PNG)

</br > 'trajectory' Function:
```
function y = fcn(u)
%u(1) : t
y = cos(u(1));
```

</br > 'error' Function:
```
function y = fcn(u)
%u(1) : x_traj - x
%u(2) : y_traj - y
%u(3) : d*
y = sqrt(u(1)^2+u(2)^2)-u(3);
```

</br > 'heading to trajectory' Function:
```
function y = fcn(u)
%u(1) : x' - x
%u(2) : y' - y
y = atan2(u(2),u(1));
```

</br > 'angle correction' Function:
```
function y = fcn(u)
%u(1) : theta* - theta
y = atan2(sin(u(1)),cos(u(1)));
```

### Differential Drive Robot
Differential Drive Robot is the subsystem in Trajectory Following.
</br > 
</br > MATLAB SIMULINK Structure of Subsystem:
</br > ![Differential Drive Robot](https://github.com/tamersum11/robotics-go-to-goal-behaviour/blob/main/png/smlkhw312.PNG)

</br > 'Subsystem/diff_drive' Function:
```
function y = fcn(u)
%u(1) : v
%u(2) : w
%u(3) : x
%u(4) : y
%u(5) : phi
 
y = [u(1)*cos(u(5)); u(1)*sin(u(5));u(2)];
```
### Output of Trajectory Following System in MATLAB SIMULINK 
</br > Output for Trajectory of cos(t):
</br > ![Output of Trajectory](/png/hw32pt.PNG)
</br > Output for Follow-Trajectory Controller:
</br > ![Output of Follow-Trajectory](/png/hw32pr.PNG)

## Output of Trajectory Following System in Python Implementation
</br > Output of Follow-Trajectory Controller for Trajectory of cos(t):
</br > ![Output of Follow-Trajectory-Python](/png/hw322.png)
