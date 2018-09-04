{% include "../math.md" %}

# Time-parameterization

The paths found in Section [Path planning](path_planning.md) are
geometric objects devoid of any timing information and, as such, cannot
be executed directly on the robot. One simple solution may consist in
time-parameterizing the path using a constant path velocity. However,
this solution is usually unsatisfactory: either the velocity is too high
and the robot will violate kinodynamic constraints at some positions on
the path, or the velocity is too low, hence inefficient. This section
presents algorithms to find *optimal* time-parameterizations, i.e. which
minimize traversal time while respecting given kinodynamic constraints.

Consider a path $$P$$ in the configuration space

<center>
$$\begin{array}{cccl}
P : & [0,1] & \rightarrow  & \mathcal{C} \\
    &   s   & \mapsto      & \bfq(s).
\end{array}$$
</center>

The objective is to find a time-parameterization $$s$$

<center>
$$\begin{array}{cccl}
s : & [0,T] & \rightarrow  & [0,1] \\
    &   t   & \mapsto      & s(t),
\end{array}$$
</center>

so that the *retimed* trajectory $$\bfq(s(t))$$ satisfies given
constraints while minimizing trajectory duration $$T$$, which is the most
important criterion in industrial robotics.

## Time-parameterization of straight segments under velocity and acceleration bounds

Velocity and acceleration bounds are the most common constraints in
industrial robotics. For a n-DOF robot, they have the form

<center>
$$\begin{array}{ccc}
|\dot q_i|  & \leq & v_i^{\max} \quad i\in [1, n] \\
|\ddot q_i|  & \leq & a_i^{\max} \quad i\in [1, n].
\end{array}$$
</center>


Paths returned by motion planners (see Section [Path
planning](path_planning.html)) are usually composed of straight
segments. A straight segment between $$\bfq_a$$ and $$\bfq_b$$ is given by
$$\bfq(s):= \bfq_a + s (\bfq_b-\bfq_a)$$. Differentiating with respect to
$$t$$, one has $$\dot\bfq = \dot s
(\bfq_b-\bfq_a)$$ and $$\ddot\bfq = \ddot s (\bfq_b-\bfq_a)$$.

Therefore, the optimization problem becomes: find the function
$$(s(t))_{t\in[0,T]}$$ such that $$T$$ is minimal and that

<center>
$$\begin{array}{ccc}   
|\dot s|  & \leq & \dot s_{\max} := \min_i \frac{v_i^{\max}}{|q_{b,i}-q_{a,i}|} \\
|\ddot s|  & \leq & \ddot s_{\max} : = \min_i \frac{a_i^{\max}}{|q_{b,i}-q_{a,i}|},
\end{array}$$
</center>

and $$s(0) = 0$$, $$s(T) = 1$$, $$\dot s(0) = 0$$, $$\dot s(T) = 0$$. Note that
the initial and final velocities are constrained to be zero so as to
ensure velocity continuity at the junction of different path segments.

This problem has a closed-form solution:

1. If $$\dot s_{\max} \geq \sqrt{\ddot s_{\max}}$$, then the optimal profile is composed of two segments: 

    i. acceleration $$\ddot s_{\max}$$ between time instants 0 and $$t_s$$;
    
    ii. deceleration $$-\ddot s_{\max}$$ between time instants $$t_s$$ and $$2t_s$$, where $$t_s := 1/\sqrt{\ddot s_{\max}}$$;

2. Else, the optimal profile is composed of three segments:

    i. acceleration $$\ddot s_{\max}$$ between time instants 0 and $$t_0$$;

    ii. zero acceleration (constant velocity) between time instants $$t_0$$ and $$t_0+t_1$$;

    iii. deceleration $$-\ddot s_{\max}$$ between time instants $$t_0+t_1$$ and $$2t_0+t_1$$, where $$t_0 := \dot s_{\max}/\ddot s_{\max}$$ and $$t_1 := 1/\dot s_{\max} - \dot s_{\max}/\ddot s_{\max}$$.

![Velocity profile for the time-parameterization of a straight
path under velocity and acceleration bounds. Left: case 1 (max
acceleration – max deceleration). Right: case 2 (max acceleration –
constant velocity – max deceleration).](../assets/planning/linear_param.png)

> #### Exercise::Time-parameterize a 2D path
>
Implement in Python the above algorithm to optimally time-parameterize
the paths found in Section [Path planning](path_planning.html) under
the following bounds $$v_1^{\max}=v_2^{\max}=0.2$$,
$$a_1^{\max}=a_2^{\max}=0.05$$.

## Time-parameterization of arbitrary paths under general second-order bounds

Second-order (kinodynamic) bounds are constraints of the form

<center>
$$\bfA(\bfq)\ddot\bfq + \dot\bfq^T \bfB(\bfq)\dot\bfq + \bff(\bfq)
\leq \mathbf{0}, \quad (*)$$
</center>

where $$\bfA(\bfq)$$, $$\bfB(\bfq)$$, and $$\bff(\bfq)$$ are, respectively, an
M x n matrix, an n x M x n tensor, and an M-dimensional vector.

Inequality (\*) is very general and may represent a large variety of
second-order systems and constraints. As an example, consider an n-DOF
manipulator with dynamics

<center>
$$\bfM(\bfq)\ddot\bfq  + \dot\bfq^T \bfC(\bfq)\dot\bfq + \bfg(\bfq) \leq
\bftau.$$
</center>

Assume that the manipulator is subject to lower and upper bounds on the
joint torques, that is, for any joint i and time t,

<center>
$$\tau_i^{\min} \leq \tau_i(t) \leq \tau_i^{\max}.$$
</center>

Clearly, these torque bounds can be put in the form of (\*) with

<center>
$$\bfA(\bfq) := \left( 
\begin{array}{r}
\bfM(\bfq)\\
-\bfM(\bfq)
\end{array}
\right), \quad 
\bfB(\bfq) := \left( 
\begin{array}{r}
\bfC(\bfq)\\
-\bfC(\bfq)
\end{array}
\right), \quad 
\bff(\bfq) := \left( 
\begin{array}{r}
\bfg(\bfq) - \bftau^{\max}\\
-\bfg(\bfq) + \bftau^{\min}
\end{array}
\right).$$
</center>

Contrary to the case of velocity and acceleration bounds and linear
paths, there is no closed-form solution in the general case of
second-order constraints and arbitrary paths. However, there is a very
efficient algorithm, first proposed by Bobrow in the 1980's and later
perfected by many others, to numerically find the optimal
time-parameterization, see Pham (2014). An implementation of the
algorithm can be found at <https://github.com/quangounet/TOPP>.

# To learn more about this topic

Hauser, K., & Ng-Thow-Hing, V. (2010). Fast smoothing of manipulator
trajectories using optimal bounded-acceleration shortcuts. In Robotics
and Automation (ICRA), 2010 IEEE International Conference on (pp.
2493-2498). IEEE.

Pham, Q. C. (2014). A general, fast, and robust implementation of the
time-optimal path parameterization algorithm. *IEEE Transactions on
Robotics*, 30(6), 1533-1540.

Lertkultanon, P., & Pham, Q. C. (2016). Time-optimal parabolic
interpolation with velocity, acceleration, and minimum-switch-time
constraints. *Advanced Robotics*, 30(17), 1095-1110.
