# Non-linear MIMO State Feedback Control

A Differential Drive Robot (DDR) system, whose model is shown below, is used in this project to apply a non-linear MIMO state feedback control.

![DDR model](./images/DDRmodel.png)

The output of the system is then

![Output](./images/output.png)

## Lie Derivatives Control

We bring the robot to the origin from an initial position first by applying Lie Derivatives. In the example below, we go from $x = [3, 3, 0]$ to the origin.

![Lie Derivative](./punto%202%20estabilizacion%20de%20las%20salidas/RobotPosition.gif)  ![Lie Derivative Tracking](./punto%202%20estabilizacion%20de%20las%20salidas/RobotTracking.gif)

## Dynamic Extension Control

With the Dynamic Extension approach, we add an additional dynamic to the system so it increases in order. In this case we add $\dot{v} = \mu$. Again we bring the robot to the origin.

![Dynamic Extension](./punto%203%20estabilizacion%20por%20extension%20dinamica/RobotPosition.gif)  ![Dynamic Extension Tracking](./punto%203%20estabilizacion%20por%20extension%20dinamica/RobotTracking.gif)

## Straight Line Tracking

Now we want the robot to follow a straight line. Tis line is parameterized as follows.

![Line parameterization](./images/lineParam.png)

Here we can see the results by applying Lie Derivatives.

![Straight Line](./punto%204%20seguimiento%20de%20linea%20recta/RobotPosition.gif)  ![Straight Line Tracking](./punto%204%20seguimiento%20de%20linea%20recta/RobotTracking.gif)

## Parameterized Curve Tracking

Finally, we use a more complex than a straight line curve for the robot to follow.

![Curve parameterization](./images/curveParam.png)

![Curve](./punto%205%20seguimiento%20de%20trayectorias/RobotPosition.gif)  ![Curve Tracking](./punto%205%20seguimiento%20de%20trayectorias/RobotTracking.gif)
