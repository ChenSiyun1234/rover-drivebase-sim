# Parameters

I use `sim/parameters.py` as the parameter registry for this rocker-bogie
simulation. `python viz/build_rover.py` generates the tables below.
**datasheet** marks manufacturer values; **assumed** marks my model settings.

Datasheet: Damiao DM-J6006-2EC User Manual V1.0, Key Parameters and CAN
protocol sections, for the 24 V hub motor:
https://f.foxtech.com/manual/DM-J6006-2EC-V1.0.pdf .

| Parameter | Value | Unit | Provenance |
|---|---|---|---|
| rated_voltage | 24.0 | V | datasheet |
| rated_torque | 4.0 | N m | datasheet |
| peak_torque | 12.0 | N m | datasheet |
| rated_current | 4.0 | A | datasheet |
| peak_current | 13.0 | A | datasheet |
| rated_speed | 150.0 | rpm | datasheet |
| no_load_speed | 240.0 | rpm | datasheet |
| gear_ratio | 6.0 | ratio | datasheet |
| motor_diameter | 0.076 | m | datasheet |
| motor_length | 0.0365 | m | datasheet |
| motor_mass | 0.335 | kg | datasheet |
| wheel_radius | 0.12 | m | assumed |
| wheel_width | 0.06 | m | assumed |
| track | 0.8 | m | assumed |
| front_x | 0.45 | m | assumed |
| middle_x | -0.05 | m | assumed |
| rear_x | -0.45 | m | assumed |
| link_y | 0.32 | m | assumed |
| bogie_x | -0.25 | m | assumed |
| bogie_z | -0.12 | m relative to chassis | assumed |
| axle_z | -0.22 | m relative to chassis | assumed |
| initial_height | 0.35 | m | assumed |
| box_size | [0.55, 0.4, 0.18] | m full lengths | assumed |
| box_z | 0.05 | m relative to chassis | assumed |
| chassis_mass | 36.0 | kg including battery/electronics | assumed |
| rocker_mass | 1.5 | kg each | assumed |
| bogie_mass | 2.5 | kg each | assumed |
| wheel_mass | 1.0 | kg each including motor | assumed |
| rocker_com | [0.02, 0.0, -0.08] | m local | assumed |
| rocker_inertia | [0.03, 0.12, 0.12] | kg m^2 diagonal | assumed |
| bogie_inertia | [0.02, 0.06, 0.06] | kg m^2 diagonal | assumed |
| link_radius | 0.025 | m | assumed |
| axle_radius | 0.02 | m | assumed |
| wheel_armature | 0.0128 | kg m^2 output referred | assumed |
| free_inertia | 0.02 | kg m^2 cylinder plus armature | assumed |
| drive_damping | 0.04 | N m s/rad | assumed |
| hinge_damping | 0.15 | N m s/rad | assumed |
| hinge_range | 45.0 | degrees each direction | assumed |
| gravity | 9.81 | m/s^2 | assumed |
| friction | [0.6, 0.005, 0.0001] | sliding, torsional m, rolling m | assumed |
| friction_sweep | [0.3, 0.6, 0.9] | sliding coefficient | assumed |
| contact_dimension | 4 | MuJoCo condim | assumed |
| contact_solref | [0.008, 1.0] | s, damping ratio | assumed |
| contact_solimp | [0.9, 0.95, 0.001] | MuJoCo impedance | assumed |
| obstacle_position | [1.2, 0.4, 0.04] | m | assumed |
| obstacle_half_size | [0.15, 0.2, 0.04] | m | assumed |
| floor_half_size | [10.0, 10.0, 0.1] | m | assumed |
| pack_voltage | 25.2 | V open circuit | assumed |
| pack_resistance | 0.04 | ohm pack and wiring | assumed |
| bus_warning | 20.0 | V reporting threshold | assumed |
| driver_undervoltage | 15.0 | V latch threshold | assumed |
| driver_current | 8.0 | A limit or immediate trip setting | assumed |
| watchdog_timeout | 0.25 | s since delivered command | assumed |
| motor_kp | 1.0 | N m/(rad/s) | assumed |
| motor_ki | 2.0 | N m/rad | assumed |
| kinematic_multiplier | 1.25 | effective track multiplier | assumed |
| yaw_kp | 2.0 | ratio | assumed |
| yaw_ki | 1.0 | 1/s | assumed |
| gyro_noise | 0.01 | rad/s standard deviation | assumed |
| governor_high | 0.85 | fraction of current setting | assumed |
| governor_low | 0.75 | fraction of current setting | assumed |
| governor_decay | 4.0 | 1/s | assumed |
| governor_recovery | 0.03 | 1/s | assumed |
| physics_dt | 0.002 | s | assumed |
| electrical_dt | 0.001 | s | assumed |
| command_period | 0.01 | s | assumed |
| can_delay | 0.001 | s per frame | assumed |
| command_drop | 0.0 | probability | assumed |
| feedback_drop | 0.0 | probability | assumed |
| seed | 7 | random seed | assumed |
| settle_time | 1.0 | s | assumed |
| duration | 6.0 | s after settling | assumed |
| straight_speed | 0.3 | m/s command | assumed |
| yaw_command | 30.0 | degrees/s command | assumed |
| stall_target | 100.0 | rad/s command with ideal wheel locks | assumed |
| command_cutoff | 2.0 | s after settling | assumed |
| steady_window | 2.0 | s at end of trial | assumed |
| battery_iterations | 28 | bisection iterations | assumed |

Derived values from the **assumed** geometry and mass registry:

| Parameter | Value | Unit | Provenance |
|---|---|---|---|
| total_mass | 50 | kg | assumed |
| wheelbase | 0.9 | m | assumed |
| rocker_front_length | 0.500899 | m | assumed |
| rocker_rear_length | 0.277308 | m | assumed |
| bogie_arm_length | 0.223607 | m | assumed |
| equal_share_load | 81.75 | N per wheel | assumed |

## Model conventions

I sum chassis, rocker, bogie and complete wheel masses; the wheels include
their motors. Wheelbase is front_x minus rear_x. Link lengths follow pivot
and axle coordinates, with the rocker pivot at the chassis origin on each
side and the bogie pivot midway between its axles. Opposite rocker angles
are coupled by an ideal differential. Chassis and wheel inertias use a
uniform box and cylinder. Equal-share load is mass times gravity divided
by six; simulated loads follow the suspension geometry and motion.

I fit the motor to the listed datasheet operating points:
Kt = rated_torque/rated_current;
Ke = rated_voltage/(no_load_speed converted to rad/s);
R_fit = (rated_voltage - Ke*(rated_speed converted to rad/s))/rated_current.
R_fit represents lumped losses. Rounded ratings give unequal Kt and Ke in
SI units, so the fit is empirical rather than energy reciprocal.
Torque is bounded by peak torque, peak current, the driver current setting
and Kt*max(0, V-Ke*abs(speed))/R_fit.
Electrical power is max(0, (torque/Kt)^2*R_fit + torque*speed), with braking
energy dissipated locally. PI integration stops when it would deepen
saturation. Current control is instantaneous. Trips latch on requested
current before clipping; undervoltage and watchdog expiry also latch.
Enable clears faults.

I solve V = pack_voltage - pack_resistance*sum(power)/V by bisection on the
high voltage branch, using fixed open-circuit voltage and pack resistance.
Lost commands hold the previous target until the watchdog expires.

CAN follows the datasheet: standard identifiers, velocity ID offset 0x200,
little endian float32 commands, and feedback with 16-bit position, 12-bit
velocity, 12-bit torque and a status nibble. I assume codec ranges of
plus/minus 12.5 rad, 30 rad/s and 12 N m. MIT-mode gain ranges are 0 to 500
and 0 to 5 (**datasheet**). I use motor IDs 1 through 6, master ID 0 and
fixed reported temperatures of 25 degrees C. Fixed frame delay represents
transport scheduling.

I start each scenario fresh, settle at zero targets and reset fault latches
before recording. Turns sweep friction with open targets, yaw PI and the
torque governor. Gyro noise is sampled per command. The governor reduces
its scale above governor_high and recovers linearly below governor_low.
The stall fixture locks wheel joints; the watchdog fixture stops commands
at command_cutoff. Rendering replays recorded poses.
