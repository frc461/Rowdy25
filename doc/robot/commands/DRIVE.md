# Drive Commands

Swerve drivetrain commands for manual and autonomous control. All live under [`commands/drive/`](../../src/main/java/io/github/frc461/rowdy25/commands/drive/).

## Commands at a Glance

- **`DriveCommand`** — Default command for the [Swerve](../subsystems/DRIVETRAIN.md) subsystem. Implements field-centric teleop drive with a configurable `DriveMode` (idle, translating, rotating / fast-rotating, branch / reef-tag / coral-station / processor / net / object auto-heading). Driver translation and rotation inputs are scaled by current elevator height via `Constants.MAX_CONTROLLED_VEL` / `Constants.MAX_CONTROLLED_ANGULAR_VEL` to prevent tipping when extended.
- **`PathfindToPoseAvoidingReefCommand`** — The reef-aware planner used by every `Swerve.pathFindTo*` helper and by all `AutoManager` segments. See [§ Reef-Avoidance Math](#reef-avoidance-math) below.
- **`DirectMoveToPoseCommand`** — Drives directly toward a target pose with PID, without invoking any planner. Used for short final approaches once the robot is close enough that obstacle avoidance is no longer relevant (e.g., post-vision alignment to a branch face).

## Reef-Avoidance Math

`PathfindToPoseAvoidingReefCommand` does *not* invoke PathPlanner's `LocalADStar`. Instead, on every scheduler tick (~20 ms) it recomputes a single **temporary target pose** that the swerve closed-loop chases, and continuously redirects this temporary target around the reef hexagon. This produces a smooth, obstacle-aware trajectory while keeping the controller entirely in-process and reactive to mid-flight pose updates from the [Localizer](../subsystems/LOCALIZER.md).

Let:

- $p_c$ — current robot pose (translation $\mathbf{c}$, heading $\theta_c$), from `swerve.localizer.getStrategyPose()`.
- $p_t$ — final target pose (translation $\mathbf{t}$, heading $\theta_t$).
- $\mathbf{r}$ — translation of the nearest reef hexagon center, from `FieldUtil.Reef.getNearestReefCenter(...)`.
- $a$ — the reef apothem `FieldUtil.Reef.REEF_APOTHEM` (distance from reef center to the midpoint of any reef face).
- $L$ — the robot length with bumpers, `Constants.ROBOT_LENGTH_WITH_BUMPERS`.

### 1. Choosing the temporary target $p^{\text{tmp}}$

`getTemporaryTargetPose(p_c)` selects one of three branches:

1. **Same sextant (no obstacle in the way).** If `RobotPoses.Reef.sameSide(p_c, p_t)` — i.e., $\mathbf{c}$ and $\mathbf{t}$ both lie on the same hex face of the reef when projected onto its sextants — there is no obstacle between them, so the temporary target is the final target:
   $$
   p^{\text{tmp}} = p_t.
   $$

2. **Robot is inside the danger ring.** If
   $$
   \lVert \mathbf{c} - \mathbf{r} \rVert \;<\; a + \tfrac{L}{1.3},
   $$
   the robot is dangerously close to the reef (the divisor 1.3 widens the apothem by a factor of $L/1.3$ — slightly less than half the robot length — to provide a safety margin). In this case the algorithm produces an *escape* waypoint: take the nearest reef-tag pose (which faces outward from the reef face), translate 2.0 m forward along that face's normal, and hold the robot's current rotation so it doesn't spin while escaping:
   $$
   p^{\text{tmp}} = \big(\mathbf{r} + R(\psi)\,(2.0,\,0),\;\theta_c\big),
   $$
   where $\psi$ is the heading of the nearest reef tag.

3. **Tangent-around-the-reef waypoint.** Otherwise the robot is outside the danger ring but on the wrong sextant. Compute two angles measured at the reef center:
   - $\alpha = \angle(\mathbf{c} - \mathbf{r})$ — direction from reef center to robot.
   - $\beta = \angle(\mathbf{t} - \mathbf{r})$ — direction from reef center to target.

   The tangent direction at $\mathbf{c}$'s side of the reef is $\alpha \pm 90°$; the sign is picked by
   $$
   \sigma = \operatorname{sign}(\beta - \alpha)
   $$
   (implemented as `Math.copySign(90.0, (β − α).getDegrees())`). This chooses the *shorter* angular path around the reef.

   The waypoint sits 2.0 m radially outward from the reef center along $\alpha$, then 1.5 m further along the tangent $\alpha + 90°\sigma$:
   $$
   \mathbf{c}_{\text{way}} = \mathbf{r} + R(\alpha)\,(2.0,\,0) + R(\alpha + 90°\sigma)\,(1.5,\,0),
   $$
   with rotation interpolated 25 % from current toward target:
   $$
   \theta^{\text{tmp}} = \theta_c \,\overset{0.25}{\longrightarrow}\, \theta_t.
   $$

### 2. Low-pass smoothing of the temporary target

Because branches 1–3 can flip discontinuously as the robot moves (e.g., when crossing a sextant boundary), the raw $p^{\text{tmp}}$ is run through a per-tick low-pass in `updateSmoothTargetPose(...)`. Let $p_s$ be the previous smoothed pose. Then:

- If $p_s$ is `null` (first tick), $p_s \leftarrow p^{\text{tmp}}$.
- If $\lVert \mathbf{s} - \mathbf{c}^{\text{tmp}} \rVert > 0.11$ m, advance $\mathbf{s}$ by 11 cm along the heading $\angle(\mathbf{c}^{\text{tmp}} - \mathbf{s})$ and slerp the rotation 25 % toward $\theta^{\text{tmp}}$.
- Otherwise snap $p_s \leftarrow p^{\text{tmp}}$.

This bounds the per-tick motion of the target by 11 cm and produces a continuous, jerk-bounded trajectory. The smoothed target $p_s$ is what the chassis controller actually chases, and is also published to `localizer.setCurrentTemporaryTargetPose(...)` for telemetry.

### 3. Velocity profile

Translation speed is a piecewise blend of a logistic ("sigmoid") curve and a clipped linear ramp, evaluated on the **distance to the smoothed target** $d = \lVert \mathbf{s} - \mathbf{c} \rVert$. Using the `EquationUtil` primitives

$$
\underbrace{f_{\text{sig}}(d;\,M,\,h,\,k) = \dfrac{M}{1 + e^{-k(d - h)}}}_{\texttt{EquationUtil.expOutput}}, \qquad
\underbrace{f_{\text{lin}}(d;\,K_p,\,b) = K_p\,d + b}_{\texttt{EquationUtil.linearOutput}},
$$

the per-tick command is

$$
v = \max\!\Big(\;\underbrace{f_{\text{sig}}\!\big(d;\,2,\;\tfrac{2}{7},\;\tfrac{15}{2}\big)}_{\text{logistic close-in floor}},\;\;\underbrace{\min\!\big(f_{\text{lin}}(d;\,10,\,-10),\;v_{\max}\big)}_{\text{linear ramp, capped}}\;\Big),
$$

with $v_{\max} = \min(\text{ctor max},\;\texttt{MAX\_CONTROLLED\_VEL}(\text{elevatorHeight}))$.

- The logistic term ($M=2\text{ m/s}$, midpoint $h = 2/7\text{ m}$, steepness $k = 7.5$) saturates at 2 m/s far from target and decays smoothly past the midpoint to ≈0.2 m/s at $d=0$. This is the deceleration profile: it never returns zero, so the controller always commands *some* forward velocity to overcome stiction.
- The linear term ($K_p = 10$, offset $-10$) is negative for $d < 1$ m and grows past the logistic at larger distances, where it is then clipped by $v_{\max}$. Taking the outer `max` means the logistic dominates close in (and prevents the linear's negative output from stopping the robot prematurely), while the linear branch dominates far out.

The resulting profile is approximately *constant cruise speed → sigmoid deceleration*, with no discontinuity at the handoff distance.

The velocity vector is steered along
$$
\hat{\mathbf{v}} = \big(\cos\gamma,\;\sin\gamma\big),\quad \gamma = \angle(\mathbf{s} - \mathbf{c}),
$$
and dispatched as a field-centric `SwerveRequest.FieldCentric.withVelocityX/Y(...)` with `withDriveRequestType(Velocity)` (closed-loop velocity on the drive motors) and `withForwardPerspective(BlueAlliance)` so the field frame is consistent across alliances.

### 4. Yaw control

Heading is a standard PID on the **shortest signed angular error**:

$$
\dot\theta_{\text{cmd}} = K_p\,e_\theta + K_d\,\dot e_\theta,\quad e_\theta = \operatorname{wrap}_{[-180°,180°]}(\theta^{\text{tmp}} - \theta_c),
$$

implemented with `PIDController.enableContinuousInput(ANGULAR_MINIMUM_ANGLE, ANGULAR_MAXIMUM_ANGLE)` to handle the wrap. The output is scaled by `MAX_CONTROLLED_ANGULAR_VEL(elevatorHeight)`.

### 5. Termination

The command finishes when all three of the following are simultaneously true against the **final** target $p_t$ (not the smoothed intermediate):

- $|x_c - x_t| < $ `TRANSLATION_TOLERANCE_TO_ACCEPT`,
- $|y_c - y_t| < $ `TRANSLATION_TOLERANCE_TO_ACCEPT`,
- $|\operatorname{wrap}_{[-180°,180°]}(\theta_c - \theta_t)| < $ `DEGREE_TOLERANCE_TO_ACCEPT`.

On `end(...)` the chassis is force-stopped and `Swerve.consistentHeading` is set to the current measured heading so the next teleop tick doesn't fight a stale heading lock.

## Features Summary

- Field-centric drive with multiple automatic heading modes (`DriveCommand`).
- Vision-based heading lock onto reef branches, tags, coral stations, processor, net, and detected game pieces.
- Reactive, continuously-re-planned reef avoidance via the math above.
- Velocity scaling tied to elevator extension to prevent tipping.

## Default Command

`DriveCommand` is installed by `RobotStates.setDefaultCommands(...)` and runs whenever no other command requires the Swerve subsystem.

## See Also

- [Swerve subsystem](../subsystems/DRIVETRAIN.md) — Owns the `DriveMode` enum and the `pathFindTo*` helpers that wrap `PathfindToPoseAvoidingReefCommand`.
- [Pathfinder utility](../autos/PATHFINDER.md) — A separate, currently unused wrapper around PathPlanner's `AutoBuilder.pathfindToPose`; kept for reference.
- [EquationUtil](../util/OTHER.md) — Provides the exponential and linear velocity primitives used in § 3.
