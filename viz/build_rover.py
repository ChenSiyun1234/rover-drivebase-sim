"""Generate a primitive rocker-bogie and its parameter reference."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.parameters import PARAMETERS, value as p


def numbers(values):
    return ' '.join(f'{v:.9g}' for v in values)


def link(parent, a, b, radius):
    ET.SubElement(parent, 'geom', type='capsule', fromto=numbers([*a, *b]),
                  size=str(radius), rgba='0.15 0.5 0.65 1', contype='0', conaffinity='0')


def build():
    root = ET.Element('mujoco', model='generic_rocker_bogie')
    ET.SubElement(root, 'compiler', angle='degree', inertiafromgeom='auto')
    ET.SubElement(root, 'option', timestep=str(p('physics_dt')),
                  gravity=f"0 0 {-p('gravity')}", integrator='implicitfast', cone='elliptic')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='960', offheight='600')
    ET.SubElement(visual, 'headlight', ambient='0.4 0.4 0.4', diffuse='0.7 0.7 0.7')
    default = ET.SubElement(root, 'default')
    ET.SubElement(default, 'geom', friction=numbers(p('friction')),
                  condim=str(p('contact_dimension')), solref=numbers(p('contact_solref')),
                  solimp=numbers(p('contact_solimp')))
    ET.SubElement(default, 'joint', damping=str(p('hinge_damping')))
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'texture', name='grid', type='2d', builtin='checker',
                  rgb1='0.22 0.25 0.29', rgb2='0.28 0.31 0.35', width='128', height='128')
    ET.SubElement(asset, 'material', name='ground', texture='grid', texrepeat='10 10')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', pos='0 -3 5', dir='0 0 -1', directional='true')
    ET.SubElement(world, 'geom', name='floor', type='plane', size=numbers(p('floor_half_size')), material='ground')
    ET.SubElement(world, 'geom', name='obstacle', type='box', pos=numbers(p('obstacle_position')),
                  size=numbers(p('obstacle_half_size')), rgba='0.75 0.52 0.25 1')
    chassis = ET.SubElement(world, 'body', name='chassis', pos=f"0 0 {p('initial_height')}")
    ET.SubElement(chassis, 'freejoint')
    ET.SubElement(chassis, 'geom', name='box', type='box', pos=f"0 0 {p('box_z')}",
                  size=numbers([v/2 for v in p('box_size')]), mass=str(p('chassis_mass')),
                  rgba='0.83 0.86 0.90 1')
    ET.SubElement(chassis, 'site', name='imu', pos='0 0 0', size='0.01')
    for side, sign in (('l', 1), ('r', -1)):
        rocker = ET.SubElement(chassis, 'body', name='rocker_'+side, pos=f"0 {sign*p('link_y')} 0")
        ET.SubElement(rocker, 'joint', name='rocker_'+side, axis='0 1 0',
                      range=numbers([-p('hinge_range'), p('hinge_range')]))
        ET.SubElement(rocker, 'inertial', pos=numbers(p('rocker_com')), mass=str(p('rocker_mass')),
                      diaginertia=numbers(p('rocker_inertia')))
        link(rocker, [0, 0, 0], [p('front_x'), 0, p('axle_z')], p('link_radius'))
        pivot = [p('bogie_x'), 0, p('bogie_z')]
        link(rocker, [0, 0, 0], pivot, p('link_radius'))
        bogie = ET.SubElement(rocker, 'body', name='bogie_'+side, pos=numbers(pivot))
        ET.SubElement(bogie, 'joint', name='bogie_'+side, axis='0 1 0',
                      range=numbers([-p('hinge_range'), p('hinge_range')]))
        ET.SubElement(bogie, 'inertial', pos='0 0 0', mass=str(p('bogie_mass')),
                      diaginertia=numbers(p('bogie_inertia')))
        for tag, x in (('f', p('front_x')), ('m', p('middle_x')), ('r', p('rear_x'))):
            parent = rocker if tag == 'f' else bogie
            local_x = x if tag == 'f' else x-p('bogie_x')
            local_z = p('axle_z') if tag == 'f' else p('axle_z')-p('bogie_z')
            dy = sign*(p('track')/2-p('link_y'))
            if tag != 'f':
                link(parent, [0, 0, 0], [local_x, 0, local_z], p('link_radius'))
            link(parent, [local_x, 0, local_z], [local_x, dy, local_z], p('axle_radius'))
            wheel = ET.SubElement(parent, 'body', name='wheel_'+side+tag,
                                  pos=numbers([local_x, dy, local_z]))
            ET.SubElement(wheel, 'joint', name='drive_'+side+tag, axis='0 1 0', limited='false',
                          damping=str(p('drive_damping')), armature=str(p('wheel_armature')))
            ET.SubElement(wheel, 'geom', name='tire_'+side+tag, type='cylinder',
                          size=numbers([p('wheel_radius'), p('wheel_width')/2]),
                          quat='0.707106781 0.707106781 0 0', mass=str(p('wheel_mass')),
                          rgba='0.09 0.10 0.12 1')
            ET.SubElement(wheel, 'geom', type='box', pos=numbers([0, sign*(p('wheel_width')/2+.001), 0]),
                          size=numbers([p('wheel_radius')*.7, .001, p('wheel_radius')*.1]),
                          mass='0', contype='0', conaffinity='0', rgba='0.2 0.65 0.8 1')
    equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'joint', name='differential', joint1='rocker_l', joint2='rocker_r',
                  polycoef='0 -1 0 0 0', solref=numbers(p('contact_solref')))
    actuator = ET.SubElement(root, 'actuator')
    for side in 'lr':
        for tag in 'fmr':
            ET.SubElement(actuator, 'motor', joint='drive_'+side+tag, gear='1', ctrllimited='true',
                          ctrlrange=numbers([-p('peak_torque'), p('peak_torque')]))
    ET.SubElement(ET.SubElement(root, 'sensor'), 'gyro', name='gyro', site='imu')
    ET.indent(root)
    ET.ElementTree(root).write(ROOT/'viz/scene/rover_scene.xml', encoding='unicode')
    write_parameters()
    print('Built primitive scene and parameter reference')


def write_parameters():
    text = '''# Parameters

I use `sim/parameters.py` as the parameter registry for this rocker-bogie
simulation. `python viz/build_rover.py` generates the tables below.
**datasheet** marks manufacturer values; **assumed** marks my model settings.

Datasheet: Damiao DM-J6006-2EC User Manual V1.0, Key Parameters and CAN
protocol sections, for the 24 V hub motor:
https://f.foxtech.com/manual/DM-J6006-2EC-V1.0.pdf .

| Parameter | Value | Unit | Provenance |
|---|---|---|---|
'''
    for name, (v, unit, source) in PARAMETERS.items():
        text += f'| {name} | {v} | {unit} | {source} |\n'
    import math
    derived = {
        'total_mass': (p('chassis_mass')+2*p('rocker_mass')+2*p('bogie_mass')+6*p('wheel_mass'), 'kg'),
        'wheelbase': (p('front_x')-p('rear_x'), 'm'),
        'rocker_front_length': (math.hypot(p('front_x'), p('axle_z')), 'm'),
        'rocker_rear_length': (math.hypot(p('bogie_x'), p('bogie_z')), 'm'),
        'bogie_arm_length': (math.hypot(p('middle_x')-p('bogie_x'), p('axle_z')-p('bogie_z')), 'm'),
    }
    derived['equal_share_load'] = (derived['total_mass'][0]*p('gravity')/6, 'N per wheel')
    text += '\nDerived values from the **assumed** geometry and mass registry:\n\n| Parameter | Value | Unit | Provenance |\n|---|---|---|---|\n'
    for name, (v, unit) in derived.items():
        text += f'| {name} | {v:.6g} | {unit} | assumed |\n'
    text += '''
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
'''
    (ROOT/'docs/PARAMETERS.md').write_text(text, encoding='utf-8')


if __name__ == '__main__':
    build()
