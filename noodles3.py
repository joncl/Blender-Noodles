import bpy, bmesh
from math import *
from enum import Enum, IntEnum
import operator
import random
import datetime
import os
import re

# Originally created for Blender 2.79
# Updated for Blender 3.0.0

class Mode(Enum):
    test = 1
    final = 2


class Generate(IntEnum):
    points = 1
    segments = 2
    curves = 3
    bones = 4
    surface = 5
    all = 6


mode = Mode.test

# final
if mode == Mode.final:
    get_data_from_file = True
    generate = Generate.all
    rope_count = 275
    rope_segment_count = 100
    min_angle = 1
    min_angle_variation_factor = 10
    rope_divisions = 8
    smooth = True
    subdivide = True
    rope_render_subdivisions = 2
    random_points = True

# test
else:
    get_data_from_file = False
    generate = Generate.points
    rope_count = 1
    rope_segment_count =100
    min_angle = 1
    min_angle_variation_factor = 10
    rope_divisions = 4
    smooth = False
    subdivide = False
    rope_render_subdivisions = 1
    random_points = True


# COMMON OPTIONS
start_height = 1250
start_height_variation = 0.05
rope_length = 1000
rope_radius = 4  # 4 inches is good for spaghetti
rope_mass = 10
temp_start_height = -100
fill_diameter = 400

# OPTIONS FOR POINTS RANDOM POINTS
# random_square_coeff = 0.000026
# random_power = 4
# random_square_coeff = 0.001
random_power = 2

# OPTIONS FOR GRID POINTS
rows = 1
columns = 1
spacing = 15

# BEHAVIOR OPTIONS
cylinder_faces = 6
segment_mass = rope_mass / rope_segment_count
rb_friction = 1                                             # friction for the cylinders
rb_bounciness = 0.5         
rb_use_margin = True            
rb_collision_margin = 0.1           
rb_damping_linear = 0.9                                     # Damping linear of the cylinders
rb_damping_angular = 0.9                                    # Damping rotation of the cylinders
rb_enable_deactivation = True           
rb_deactivate_linear = 1750                                 # 25/100 - Linear Velocity below which simulation stops simulating object
rb_deactivate_angular = 175                                 # 25/100 - Angular Velocity below which simulation stops simulating object
# collision_stickiness = 0.2

# NAME PREFIXES
rope_prefix = 'Rope_'
rope_curve_prefix = rope_prefix + 'Curve_'
rope_curve_data_prefix = rope_curve_prefix + 'Data_'
rope_hook_prefix = 'Hook_'
rope_surface_prefix = rope_prefix + 'Surface_'
rope_surface_data_prefix = rope_surface_prefix + 'Data_'
bevel_prefix = 'Bevel_'
rope_group_prefix = 'RopeChain_'
segment_prefix = 'Link_'
hook_prefix = 'Hook_'
armature_data_prefix = 'Armature_'
armature_rig_prefix = armature_data_prefix + 'Rig_'
bone_prefix = 'Bone_'
rope_subsurf_prefix = "Subsurf_"
temp_prefix = 'Temp_'

objects_to_link_list = []
coords_file = os.path.join(os.path.dirname(bpy.data.filepath), 'data_coords.txt')
angles_file = os.path.join(os.path.dirname(bpy.data.filepath), 'data_angles.txt')


def unselect_all():
    for o in bpy.context.scene.objects:
        o.select_set(False)


def get_name(prefix, index1, index2=None):
    if index2 is None:
        index2 = ''
    else:
        index2 = '-' + str(index2).zfill(3)
    return prefix + str(index1).zfill(3) + index2


def select(name, make_selected=True, make_active=False):
    o = bpy.data.objects[name]
    if make_selected is True:
        o.select_set(make_selected)
        if make_selected is True:
            bpy.context.view_layer.objects.active = o
    else:
        bpy.context.view_layer.objects.active = None
    return o


def unselect(name):
    select(name, False)


def set_edit_mode():
    bpy.ops.object.mode_set(mode='EDIT')


def set_object_mode():
    bpy.ops.object.mode_set(mode='OBJECT')


def set_pose_mode():
    bpy.ops.object.mode_set(mode='POSE')


def delete_cylinder(name):
    unselect_all()
    select(name)
    bpy.ops.object.delete(use_global=False)


def create_rope_group(index):
    name = get_name(rope_group_prefix, index)
    bpy.ops.group.create(name=name)


def get_random_coords():
    fill_diameter_tol = 0.05
    min_distance_coeff_adjust = 0.10                        # adjust minimum distance to fit inside fill_diameter
    fill_radius = fill_diameter / 2
    normalized_fill_radius = (fill_radius ** 2) / fill_radius

    def distance(coord1, coord2):
        return abs(hypot(coord1[0] - coord2[0], coord1[1] - coord2[1]))

    def get_new_coord_list():
        coord_list = []

        point_count = 0
        point_repeat = 0
        square_x = 0

        circle_r_boundary = 0.9                             # keep-out for random radius (provides some variation along circle_r)
        min_distance_factor = 3                             # minimum distance factor between ropes
        min_distance = min_distance_factor * rope_radius    # minimum distance between ropes

        def check_distance(tmp_coord):
            if len(coord_list) < 1:
                return True
            tmp_distance_list = []
            for c in coord_list:
                dist = distance(c, tmp_coord)
                # dist = abs(hypot(c[0] - new_coord[0], c[1] - new_coord[1]))
                tmp_distance_list.append(dist)

            min_dist = min(tmp_distance_list)
            # print('{0} min_dist: {1}'.format(' ' * 80, str(min_dist)[:5]))
            if min_dist < min_distance:
                return False
            else:
                return True

        while True:
            angle = random.random() * pi * 2
            # circle_r = random_square_coeff * square_x ** random_power
            circle_r = (square_x ** 2) / normalized_fill_radius
            ran = random.random()
            radius = circle_r * (
                circle_r_boundary + ran - circle_r_boundary * ran)  # random radius between circle_r_boundary and circle_r
            # print('point_count: {0}  square_x: {1}  circle_r: {2}  radius: {3}  repeat: {4}'.format(
            # str(point_count)[:5],
            # str(square_x),
            #     str(circle_r),
            #     str(radius)[:5],
            #     str(point_repeat)))
            x = radius * cos(angle)
            y = radius * sin(angle)
            # get random start height
            start_height_adjust = start_height_variation * random.random() * start_height
            coord = tuple(map(operator.sub, (x, y, start_height), (0, 0, start_height_adjust)))
            if check_distance(coord):
                coord_list.append(coord)
                point_count += 1
                point_repeat = 0
                if point_count == rope_count:
                    break
            else:
                if point_repeat > 9:
                    square_x += 1
                    min_distance_factor += min_distance_coeff_adjust
                    min_distance = min_distance_factor * rope_radius
                    point_repeat = -1
                point_repeat += 1
                continue

        # print('COORD_LIST: ' + str(coord_list))
        return coord_list

    fill_tolerance = fill_radius * fill_diameter_tol
    print('TOLERANCE: ' + str(fill_tolerance))

    while True:
        distance_list = []
        return_coord_list = get_new_coord_list()
        if rope_count == 1:
            break
        for new_coord in return_coord_list:
            distance_list.append(distance((0, 0, start_height), new_coord))

        max_dist = max(distance_list)
        difference = fill_radius - max_dist
        difference_percent = difference / fill_radius
        print()
        print('MAX_DIST: ' + str(max_dist) + '  DIFFERENCE: ' + (str(difference)) + '  DIFFERENCE_PERCENT: ' + str(
            difference_percent))
        if abs(difference) < fill_tolerance:
            print('break')
            break

        adjust_factor = 1 + difference_percent
        if adjust_factor < 0:
            print('NEGATIVE ADJUST_FACTOR: ' + str(adjust_factor))
            adjust_factor = 0.5

        min_distance_coeff_adjust *= adjust_factor
        print('min_distance_coeff_adjust: ' + str(min_distance_coeff_adjust))

    print('min_distance_coeff_adjust: ' + str(min_distance_coeff_adjust))
    return return_coord_list


def hide(o):
    o.hide_set(True)
    bpy.data.objects[o.name].hide_render = True


def hide_rope_objects(prefix):
    for i in range(rope_count):
        name = get_name(prefix, i)
        select(name)
        bpy.context.object.hide = True
        bpy.data.objects[name].hide_render = True


def create_empty(coord, rope_number, temp=False):
    name_prefix = get_temp_prefix(temp) + rope_prefix
    name = name_prefix + str(rope_number).zfill(3)
    location = coord
    # location = tuple(map(operator.sub, coord, (0, 0, temp_start_height)))
    bpy.ops.object.empty_add(type='PLAIN_AXES', radius=4.0, align='WORLD', location=location)
    # bpy.ops.transform.resize(value=(100, 100, 100))
    empty = bpy.context.object
    empty.name = name
    unselect(name)
    return empty


def get_object(name):
    return bpy.data.objects[name]


def parent(o, parent_name):
    p = get_object(parent_name)
    mw = o.matrix_world.copy()
    o.parent = p
    o.matrix_world = mw
    return p


def reset_location(child_object, child_coord, parent_object):
    child_object.location = tuple(map(operator.sub, child_coord, parent_object.location))


def create_cylinder(rope_number, segment_number, coord):
    # creation of the cylinder and add rigid body characteristics
    bpy.ops.mesh.primitive_cylinder_add(vertices=cylinder_faces, radius=rope_radius,
                                        depth=rope_length / rope_segment_count,
                                        end_fill_type='NOTHING',
                                        calc_uvs=True,
                                        enter_editmode=False,
                                        align='WORLD',
                                        location=coord)
    name = get_name(temp_prefix + segment_prefix, rope_number, segment_number)
    # bpy.context.scene.objects.active.name = name
    bpy.ops.rigidbody.objects_add(type='ACTIVE')
    cylinder = bpy.context.object
    cylinder.name = name
    cylinder.data.name = get_name(temp_prefix + 'Cylinder_', rope_number, segment_number)
    rb = cylinder.rigid_body
    rb.enabled = True  # Dynamic checked
    rb.kinematic = False  # Animated unchecked
    rb.mass = segment_mass
    rb.use_margin = rb_use_margin
    rb.collision_margin = rb_collision_margin
    rb.friction = rb_friction
    rb.restitution = rb_bounciness
    rb.use_deactivation = rb_enable_deactivation
    rb.deactivate_linear_velocity = rb_deactivate_linear
    rb.deactivate_angular_velocity = rb_deactivate_angular
    rb.linear_damping = rb_damping_linear
    rb.angular_damping = rb_damping_angular
    rb.collision_shape = 'CYLINDER'

    # add rigid body constraint
    bpy.ops.rigidbody.constraint_add()
    cylinder.rigid_body_constraint.type = 'GENERIC'

    # disable rigid body constraint on last segment
    if segment_number == rope_segment_count - 1:
        cylinder.rigid_body_constraint.enabled = False

    # # add collision
    # bpy.ops.object.modifier_add(type='COLLISION')
    # cylinder.collision.stickiness = collision_stickiness

    unselect(name)

    return cylinder


def create_temp_cylinders(segment_coord_list, rope_number, parent_name):
    temp_cylinder_list = []
    for segment_number, segment_coord in enumerate(segment_coord_list):
        # print('SEGMENT_COORD_LIST: ' + str(segment_coord_list))
        cylinder = create_cylinder(rope_number, segment_number, segment_coord)
        parent_object = parent(cylinder, parent_name)
        reset_location(cylinder, segment_coord, parent_object)
        temp_cylinder_list.append(cylinder)
    return temp_cylinder_list


def create_rope_curve(coord_list, rope_number, temp=False):
    # create the curve
    # print('COORD_LIST: ' + str(coord_list))
    tmp_prefix = get_temp_prefix(temp)
    curve_data_name = get_name(tmp_prefix + rope_curve_data_prefix, rope_number)
    curve_data = bpy.data.curves.new(name=curve_data_name, type='CURVE')
    curve_data.dimensions = '3D'

    curve_name = get_name(temp_prefix + rope_curve_prefix, rope_number)
    curve = bpy.data.objects.new(curve_name, curve_data)
    starting_coord = coord_list[0]
    curve.location = starting_coord
    bpy.context.scene.collection.objects.link(curve)
    select(curve_name)

    polyline = curve_data.splines.new('POLY')
    polyline.points.add(len(coord_list) - 1)
    for i, coord in enumerate(coord_list):
        # print('i: ' + str(i) + '   coord: ' + str(coord))
        polyline.points[i].co = (0, 0, coord[2] - starting_coord[2], 1)

    # add hook modifiers
    for i in range(rope_segment_count):
        hook_name = get_name(hook_prefix, i)
        mod = curve.modifiers.new(hook_name, type='HOOK')
        # mod.falloff_type = 'NONE'

    # parent rope curve to rope
    name_prefix = get_temp_prefix(temp) + rope_prefix
    rope_name = get_name(name_prefix, rope_number)
    rope = parent(curve, rope_name)

    # set location
    reset_location(curve, coord_list[0], rope)

    unselect(curve_name)

    return curve


def update_rb_constraints(rope_number, temp=False):
    if get_data_from_file:
        if not os.path.isfile(angles_file):
            raise Exception('{0} not found'.format(angles_file))
        with open(angles_file) as f:
            lines = f.read().splitlines()
        angles_list = lines[0].split(',')
        print(angles_list)


    name_prefix = get_temp_prefix(temp) + segment_prefix
    for i in range(rope_segment_count - 1):
        name1 = get_name(name_prefix, rope_number, i)
        name2 = get_name(name_prefix, rope_number, i + 1)
        o = bpy.data.objects[name1]
        # update of the generic constraint between current 'ghost' cylinder and previous one,
        # done in a second pass because of reference to cylinder+1
        o.rigid_body_constraint.type = 'GENERIC'
        o.rigid_body_constraint.use_limit_lin_x = True
        o.rigid_body_constraint.use_limit_lin_y = True
        o.rigid_body_constraint.use_limit_lin_z = True
        o.rigid_body_constraint.use_limit_ang_x = True
        o.rigid_body_constraint.use_limit_ang_y = True
        o.rigid_body_constraint.use_limit_ang_z = True
        o.rigid_body_constraint.limit_lin_x_lower = 0
        o.rigid_body_constraint.limit_lin_x_upper = 0
        o.rigid_body_constraint.limit_lin_y_lower = 0
        o.rigid_body_constraint.limit_lin_y_upper = 0
        o.rigid_body_constraint.limit_lin_z_lower = 0
        o.rigid_body_constraint.limit_lin_z_upper = 0

        if get_data_from_file:
            ang = float(angles_list[i])
        else:
            ang = min_angle * (1 + min_angle_variation_factor * sqrt(random.random()))
            ang = 2 * pi * ang / 360

        # ang = min_angle * (1 + min_angle_variation_factor * sqrt(random.random()))
        o.rigid_body_constraint.limit_ang_x_lower = ang * -1
        o.rigid_body_constraint.limit_ang_x_upper = ang
        o.rigid_body_constraint.limit_ang_y_lower = ang * -1
        o.rigid_body_constraint.limit_ang_y_upper = ang
        o.rigid_body_constraint.limit_ang_z_lower = ang * -1
        o.rigid_body_constraint.limit_ang_z_upper = ang
        o.rigid_body_constraint.object1 = bpy.data.objects[name2]
        o.rigid_body_constraint.object2 = bpy.data.objects[name1]


def select_only(item_number, item_collection, range_count):
    for i in range(range_count):
        if item_number == i:
            item_collection[i].select = True
        else:
            item_collection[i].select = False


def get_temp_prefix(temp):
    return temp_prefix if temp else ''


def hook_vertices(rope_number, temp=False):
    tmp_prefix = get_temp_prefix(temp)
    rope_curve_name = get_name(tmp_prefix + rope_curve_prefix, rope_number)
    rope_curve = select(rope_curve_name)

    # hook select only works on edit mode so:
    set_edit_mode()

    for vertex_number, rope_modifier in enumerate(rope_curve.modifiers):
        if rope_modifier.type == 'HOOK':
            # select only the current vertex before running hook_select
            rope_splines = rope_curve.data.splines[0].points
            select_only(vertex_number, rope_splines, rope_segment_count)
            bpy.ops.object.hook_assign(modifier=rope_modifier.name)
            segment_name = get_name(get_temp_prefix(temp) + segment_prefix, rope_number, vertex_number)
            bpy.context.object.modifiers[rope_modifier.name].object = bpy.data.objects[segment_name]
            bpy.ops.object.hook_reset(modifier=rope_modifier.name)

    set_object_mode()
    unselect(rope_curve_name)


def hide_segments(segment_count, rope_number):
    for i in range(segment_count):
        name = get_name(segment_prefix, rope_number, i)
        bpy.data.objects[name].hide = True
        bpy.data.objects[name].hide_render = True
    unselect_all()


def create_bones(starting_coord, rope_number):
    # print('starting_coord: ' + str(starting_coord))
    bpy.ops.object.armature_add(radius=1.0, enter_editmode=True, align='WORLD', location=starting_coord, rotation=(0, 0, 0))
    rig = bpy.context.view_layer.objects.active
    rig.name = get_name(temp_prefix + armature_rig_prefix, rope_number)
    arm = rig.data
    arm.name = get_name(temp_prefix + armature_data_prefix, rope_number)
    # arm.show_names = True
    # rig.show_x_ray = True
    # arm.show_axes = True

    # create bones
    for bone_number in range(rope_segment_count - 1):
        bone = arm.edit_bones[bone_number]
        bone.name = get_name(bone_prefix, bone_number)
        if bone_number == 0:
            # parent bone
            bone.tail = (0, 0, -rope_segment_length)
            bpy.ops.armature.extrude()
        else:
            # child bone
            parent_bone = arm.edit_bones[bone_number - 1]
            bone.head = parent_bone.tail
            bone.tail = (bone.head[0], bone.head[1], bone.head[2] - rope_segment_length)
            if bone_number < (rope_segment_count - 2):
                bpy.ops.armature.extrude()

    # add spline IK constraint to curve
    last_bone = arm.edit_bones[rope_segment_count - 2]
    last_bone.select = True
    set_pose_mode()
    bpy.ops.pose.constraint_add(type='SPLINE_IK')
    constraint = bpy.context.object.pose.bones[-1].constraints["Spline IK"]
    constraint.target = bpy.data.objects[get_name(temp_prefix + rope_curve_prefix, rope_number)]
    constraint.chain_count = rope_segment_count - 1
    constraint.use_even_divisions = True
    constraint.y_scale_mode = 'FIT_CURVE'
    set_object_mode()

    # parent armature to rope
    rope_name = get_name(temp_prefix + rope_prefix, rope_number)
    parent(rig, rope_name)

    unselect(rig.name)

    return rig


def create_rope_surface(coord_list, rope_number):
    starting_coord = coord_list[0]
    # print('starting_coord: ' + str(starting_coord))
    rope_surface_name = get_name(temp_prefix + rope_surface_prefix, rope_number)
    rope_surface_length = rope_length + 2 * rope_radius - rope_segment_length
    bpy.ops.mesh.primitive_round_cube_add(rotation=(0, 0, 0), odd_axis_align=False,
                                              location=(starting_coord[0], starting_coord[1],
                                              starting_coord[2] - rope_surface_length / 2 + rope_radius),
                                              radius=rope_radius, size=(0, 0, rope_surface_length), arc_div=rope_divisions,
                                              lin_div=0, div_type='CORNERS')
    rope_surface = bpy.context.object
    rope_surface.name = rope_surface_name
    rope_surface_data_name = get_name(temp_prefix + rope_surface_data_prefix, rope_number)
    rope_surface.data.name = rope_surface_data_name
    if smooth:
        bpy.ops.object.shade_smooth()

    # set origin location
    bpy.context.scene.cursor.location = starting_coord
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')

    # # split capsule
    set_edit_mode()
    for i, coord in enumerate(coord_list):
        if 0 < i < (len(coord_list) - 1):
            bpy.ops.mesh.select_all(action='SELECT')
            bpy.ops.mesh.bisect(plane_co=coord, plane_no=(0, 0, 1))

        bpy.ops.mesh.select_all(action='DESELECT')
    set_object_mode()

    # add subdivision surface modifier
    if subdivide:
        modifier_name = get_name(rope_surface_prefix, rope_number)
        rope_surface.modifiers.new(modifier_name, type='SUBSURF')
        rope_surface.modifiers[modifier_name].levels = 0
        rope_surface.modifiers[modifier_name].render_levels = rope_render_subdivisions

    # parent the cylinder created to the rope node
    rig_name = get_name(temp_prefix + armature_rig_prefix, rope_number)
    select(rope_surface.name)
    select(rig_name, True, True)
    bpy.ops.object.parent_set(type='ARMATURE_AUTO', keep_transform=False)
    unselect(rig_name)
    unselect(rope_surface_name)

    return rope_surface


class Timer:
    def __init__(self, msg=None):
        self.t1 = datetime.datetime.now()
        if msg is not None:
            print(msg)

    def mark(self, start_time=None, msg=''):
        t2 = datetime.datetime.now()
        elapsed = 'Elapsed:  '.rjust(12) + str((t2 - start_time).total_seconds())[:-3].rjust(
            6) if start_time is not None else ''
        print(
            msg.rjust(5) + elapsed.rjust(9) + 'Delta:  '.rjust(12) + str((t2 - self.t1).total_seconds())[:-3].rjust(5))

    def get_start_time(self):
        return self.t1


def get_rope_coord_list(starting_coord):
    rope_coord_list = []
    for segment_number in range(rope_segment_count):
        segment_coord = (
            starting_coord[0], starting_coord[1],
            starting_coord[2] - segment_number * rope_length / rope_segment_count)
        rope_coord_list.append(segment_coord)
    return rope_coord_list


def get_coord_list():
    # get starting coords
    if get_data_from_file:
        if not os.path.isfile(coords_file):
            raise Exception('{0} not found'.format(coords_file))
        starting_coord_list = []
        with open(coords_file) as f:
            lines = f.read().splitlines()
        for l in lines:
            coords = l.split(',')
            starting_coord_list.append((float(coords[0]), float(coords[1]), float(coords[2])))
            # print(starting_coord_list)

    elif random_points:
        starting_coord_list = get_random_coords()
    else:
        starting_coord_list = []
        for r in range(rows):
            y = spacing * (2 * r - rows + 1) / 2
            for c in range(columns):
                x = spacing * (2 * c - columns + 1) / 2
                coord = (x, y, start_height)
                starting_coord_list.append(coord)

    # add segment coords to make final coord list - all coords are relative to 0, 0, 0
    coord_list = []
    for rope_number in range(rope_count):
        starting_coord = starting_coord_list[rope_number]
        print(starting_coord)
        rope_coord_list = get_rope_coord_list(starting_coord)
        coord_list.append(rope_coord_list)

    return coord_list


def delete_temp_objects(temp_object_list):
    for o in temp_object_list:
        # obj = bpy.data.objects['Cube']
        #bpy.data.scenes[0].collection.objects.unlink(o)
        bpy.data.objects.remove(o, do_unlink=True)


def link_objects():
    scene = bpy.context.scene
    for o in objects_to_link_list:
        scene.objects.link(o)
    scene.update()


def create_temp_objects(starting_coord):
    object_list = []
    rope_number = 0
    rope_coord_list = get_rope_coord_list(starting_coord)

    # empty
    temp_empty = create_empty((0, 0, 0), rope_number, temp=True)
    object_list.extend([temp_empty])

    if generate == Generate.points:
        return object_list

    if generate >= Generate.segments:
        # cylinder
        temp_cylinder_list = create_temp_cylinders(rope_coord_list, rope_number, temp_empty.name)
        object_list.extend(temp_cylinder_list)

        # create rb constraints
        update_rb_constraints(rope_number, temp=True)

    if generate >= Generate.curves:
        # curve
        temp_curve = create_rope_curve(rope_coord_list, rope_number, temp=True)
        object_list.extend([temp_curve])

        # hook rope to segments
        hook_vertices(rope_number, temp=True)

    if generate >= Generate.bones:
        # rig
        rig = create_bones(starting_coord, rope_number)
        object_list.extend([rig])

    if generate >= Generate.surface:
        # surface
        rope_surface = create_rope_surface(rope_coord_list, rope_number)
        object_list.extend([rope_surface])

    return object_list


def create_ropes():
    temp_object_list = create_temp_objects((0, 0, temp_start_height))
    coord_list = get_coord_list()

    timer1 = Timer()

    # copy each rope and set it up
    for rope_number, coord in enumerate(coord_list):
        timer2 = Timer()

        # select everything in the temp rope
        for o in temp_object_list:
            o.select_set(True)

        bpy.ops.object.duplicate(linked=False)
        # bpy.ops.object.duplicate_move(OBJECT_OT_duplicate={"linked":False, "mode":'TRANSLATION'}) #, TRANSFORM_OT_translate={"value": coord[0]})
        unselect_all()

        # rename the duplicated rope
        def get_temp_ext(i):
            return '.' + str(i).zfill(3)

        # rope node
        temp_rope_name = get_name(temp_prefix + rope_prefix, 0) + get_temp_ext(1)
        rope_name = get_name(rope_prefix, rope_number)
        rope = get_object(temp_rope_name)
        rope.name = rope_name
        rope.location = coord[0]
        if generate > Generate.points:
            hide(rope)

        if generate >= Generate.segments:
            # rope segment
            for segment_number in range(rope_segment_count):
                temp_segment_name = get_name(temp_prefix + segment_prefix, 0, segment_number) + get_temp_ext(1)
                segment_name = get_name(segment_prefix, rope_number, segment_number)
                segment_data_name = get_name('Cylinder_', rope_number, segment_number)
                segment = get_object(temp_segment_name)
                segment.name = segment_name
                segment.data.name = segment_data_name
                if generate > Generate.segments:
                    hide(segment)

        if generate >= Generate.curves:
            # rope curve
            temp_rope_curve_name = get_name(temp_prefix + rope_curve_prefix, 0) + get_temp_ext(1)
            rope_curve_name = get_name(rope_curve_prefix, rope_number)
            rope_curve_data_name = get_name(rope_curve_data_prefix, rope_number) + get_temp_ext(2)
            rope_curve = get_object(temp_rope_curve_name)
            rope_curve.name = rope_curve_name
            rope_curve.data.name = rope_curve_data_name
            if generate > Generate.curves:
                hide(rope_curve)

        if generate >= Generate.bones:
            # armature rig
            temp_arm_rig_name = get_name(temp_prefix + armature_rig_prefix, 0) + get_temp_ext(1)
            arm_rig_name = get_name(armature_rig_prefix, rope_number)
            rig = get_object(temp_arm_rig_name)
            rig.name = arm_rig_name
            arm_data_name = get_name(armature_data_prefix, rope_number)
            rig.data.name = arm_data_name
            if generate > Generate.bones:
                hide(rig)

        if generate >= Generate.surface:
            # rope surface
            temp_rope_surface_name = get_name(temp_prefix + rope_surface_prefix, 0) + get_temp_ext(1)
            rope_surface_name = get_object(temp_rope_surface_name)
            rope_surface_data_name = get_name(rope_surface_data_prefix, rope_number)
            rope_surface = rope_surface_name
            rope_surface.name = get_name(rope_surface_prefix, rope_number)
            rope_surface.data.name = rope_surface_data_name
            # add material
            # print('materials: ' + str(bpy.data.materials.items()))
            noodle_material = bpy.data.materials['Noodle']
            if noodle_material:
                material = bpy.data.materials['Noodle']
                rope_surface.data.materials.append(material)

        timer2.mark(timer1.get_start_time(), str(rope_number) + ':')

    delete_temp_objects(temp_object_list)


rope_segment_length = rope_length / rope_segment_count

if not random_points:
    rope_count = rows * columns

try:
    create_ropes()
except Exception as e:
    print('ERROR: {0}'.format(e))
