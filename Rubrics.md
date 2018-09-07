# Rubric questions

## The Vehicle Model

```
x<sub>i+1</sub> = (x<sub>i</sub> + v<sub>i</sub> * cos(psi<sub>i</sub> * dt))

y<sub>i+1</sub> = (y<sub>i</sub> + v<sub>i</sub> * sin(psi<sub>i</sub> * dt))

psi<sub>i+1</sub> = psi<sub>i</sub> - v<sub>i</sub> * delta<sub>i</sub> / Lf * dt

v<sub>i+1</sub> = v<sub>i</sub> + a<sub>i</sub> * dt

cte<sub>i+1</sub> = f(x<sub>i</sub>) - y<sub>i</sub> + v<sub>i</sub> * sin(epsi<sub>i</sub>) * dt

epsi<sub>i+1</sub> = psi<sub>i</sub> - psides<sub>i</sub> - v<sub>i</sub> * delta<sub>i</sub> / Lf * dt

```

Where:

`x`, `y`: The position of the car 

`v`: The car's velocity 

`psi`: The direction of the car's heading

`cte`: The cross-track error

`epsi`: The direction error

`Lf`: The distance between the center of mass of the vehicle and the front wheels


## Update the state

To update the state, man should first transform the way points into the car coordinate, which needs the following equations:

```

x_path = (ptsx - x) * cos(psi) + (ptsy - y) * sin(psi)

y_path = (ptsy - y) * cos(psi) - (ptsx - x) * sin(psi)

```

Where:

`x_path`, `y_path`: Transformed way points in car coordinate

Cause the initial position and the direction of the car in car's coordinate is all zero, so the initial state, which will be given to the mpc solver should be:

`0, 0, 0, v, cte, epsi`

## Polynomial Fitting

To fit the way points into polynomial function, I used a function called polyfit, which can be found in the main.cpp file, line 48.

The input of this function should be the transformed way points from last step, the output shoule be a vector containing the coefficients of every term.

## Latency Problem

To fix the latency, I defined a parameter `delay`, which represent the delay time in second. And I let the car 'run' for a delay time and take this state as initial state, with the following functions:

```

x_next = v * delay

y_ next = 0

psi_next = -(v * steer_value *delay / Lf)

v_next = v + throttle_value * delay

cte_next = polyevale(coeffs, 0) + v * sin(-atan(coeffs[1])) * delay

epsi_next = -atan(coeffs[1]) - v * atan(coeffs[1]) * delay / Lf

```

And the input state of our mpc solver should be:

`x_next, y_next, psi_next, v_next, cte_nect, epsi_next`

