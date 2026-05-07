import os
import glob
import xml.etree.ElementTree as ET
import bpy
import numpy as np
from mathutils import Vector

# 全局变量，用于在解析相对网格路径时参考 URDF 所在目录
URDF_DIR = None


def find_rootlinks(joints):
    """Return all links that don't occur as child in any joint"""
    parents = []
    children = []
    for joint in joints:
        parents.append(joint.find('parent').attrib['link'])
        children.append(joint.find('child').attrib['link'])

    rootlinks = list(set(parents) - set(children))
    return rootlinks


def find_childjoints(joints, link):
    """Returns all joints that contain the link as parent"""
    childjoints = []
    for joint in joints:
        if joint.find('parent').attrib['link'] == link:
            childjoints.append(joint)
    return childjoints


def select_only(blender_object):
    """Selects and actives a Blender object and deselects all others"""
    bpy.ops.object.select_all(action='DESELECT')
    bpy.context.view_layer.objects.active = blender_object
    blender_object.select_set(True)


def add_next_empty(empty, joint):
    """Duplicates the empty and applies the transform specified in the joint"""
    select_only(empty)
    
    bpy.ops.object.duplicate()
    new_empty = bpy.context.active_object
    new_empty.name = 'TF_' + joint.attrib['name']

    origin = joint.find('origin')
    if origin is not None:
        if 'xyz' in origin.attrib:
            translation = [float(s) for s in origin.attrib['xyz'].split()]
            bpy.ops.transform.translate(value=translation, orient_type='LOCAL')

        bpy.context.scene.cursor.matrix = new_empty.matrix_world
        
        if 'rpy' in origin.attrib:
            roll, pitch, yaw = [float(s) for s in origin.attrib['rpy'].split()]    
            bpy.ops.transform.rotate(value=roll, orient_axis='X', orient_type='CURSOR')
            bpy.ops.transform.rotate(value=pitch, orient_axis='Y', orient_type='CURSOR')
            bpy.ops.transform.rotate(value=yaw, orient_axis='Z', orient_type='CURSOR')

    bpy.context.view_layer.update()
    return new_empty


def parse_mesh_filename(mesh_filename):
    """This function will return the mesh path if it can be found, else throw an error"""
    if not mesh_filename:
        return None

    mesh_filename = mesh_filename.strip()

    # 直接存在的路径
    if os.path.exists(mesh_filename):
        return mesh_filename
    
    if 'package://' in mesh_filename:
        filepath_package = mesh_filename.replace('package://', '')
        filepath_split = filepath_package.split('/')
        package_name = filepath_split[0]
        filepath_in_package = os.path.join(*filepath_split[1:])
        
        # 按照标准 URDF 包结构查找：
        # 标准结构：<robot_name>/
        #           ├─urdf/
        #           ├─meshes/
        #           └─xml/
        if URDF_DIR:
            # 策略1: 假设 URDF 在 urdf/ 目录中，上级目录就是 robot_name
            # 直接查找 <robot_name>/meshes/ 中的文件
            robot_root = os.path.dirname(URDF_DIR)
            candidate = os.path.normpath(os.path.join(robot_root, filepath_in_package))
            if os.path.exists(candidate):
                print(f'Found mesh at (standard structure): {candidate}')
                return candidate
            
            # 策略2: 兼容在 URDF_DIR 同级查找 meshes 的情况
            candidate = os.path.normpath(os.path.join(URDF_DIR, '..', filepath_in_package))
            if os.path.exists(candidate):
                print(f'Found mesh at (sibling meshes): {candidate}')
                return candidate
            
            # 策略3: 尝试 <robot_name>/<package_name>/ 结构
            candidate = os.path.normpath(os.path.join(robot_root, package_name, filepath_in_package))
            if os.path.exists(candidate):
                print(f'Found mesh at (package-named subdirectory): {candidate}')
                return candidate
            
            # 策略4: 向上两级查找（适应嵌套更深的结构）
            grandparent = os.path.dirname(robot_root)
            candidate = os.path.normpath(os.path.join(grandparent, filepath_in_package))
            if os.path.exists(candidate):
                print(f'Found mesh at (two levels up): {candidate}')
                return candidate
            
            candidate = os.path.normpath(os.path.join(grandparent, package_name, filepath_in_package))
            if os.path.exists(candidate):
                print(f'Found mesh at (two levels up with package): {candidate}')
                return candidate
        
        # 策略5: 如果有 ROS_PACKAGE_PATH，则使用标准 ROS 查找方式
        ros_package_paths = os.environ.get('ROS_PACKAGE_PATH')
        if ros_package_paths:
            ros_package_paths = ros_package_paths.split(':')
            print(f'ROS_PACKAGE_PATH: {ros_package_paths}')

            for ros_package_path in ros_package_paths:
                # 在 ROS_PACKAGE_PATH 中查找同名的包目录
                for package_path in glob.glob(ros_package_path + '/**/' + package_name, recursive=True):
                    filepath = os.path.join(package_path, filepath_in_package)
                    if os.path.exists(filepath):
                        print(f'Found mesh via ROS_PACKAGE_PATH at: {filepath}')
                        return filepath
        else:
            print(
                f'Warning: URDF file references a mesh file from a ROS package:\n'
                f'{mesh_filename}\n'
                'Attempting to resolve using standard URDF package structure.'
            )
    
    # 如果是相对路径（相对于 URDF 文件），尝试用 URDF_DIR 解析
    if not os.path.isabs(mesh_filename):
        if URDF_DIR:
            candidate = os.path.normpath(os.path.join(URDF_DIR, mesh_filename))
            if os.path.exists(candidate):
                return candidate

        # 最后尝试相对于当前工作目录
        candidate2 = os.path.normpath(os.path.join(os.getcwd(), mesh_filename))
        if os.path.exists(candidate2):
            return candidate2

    print('Cant find the mesh file :(', mesh_filename)
    return None


def load_mesh(mesh):
    mesh_filename = mesh.attrib.get('filename') or mesh.attrib.get('file')
    mesh_path = parse_mesh_filename(mesh_filename)

    if not mesh_path:
        raise RuntimeError(f"无法解析网格路径: {mesh_filename}")

    mesh_path = os.path.normpath(mesh_path)

    # 记录修改前场景对象，方便识别本次导入新建的对象
    pre_objs = set(bpy.context.scene.objects)

    ext = os.path.splitext(mesh_path)[1].lower()

    try:
        if ext == '.stl':
            # Blender 4.1+ 使用 wm.stl_import
            try:
                bpy.ops.wm.stl_import(filepath=mesh_path)
            except Exception:
                try:
                    bpy.ops.import_mesh.stl(filepath=mesh_path)
                except Exception as e:
                    raise RuntimeError(f"STL 导入失败: {e}")
        elif ext == '.obj':
            # Blender 4.1+ 使用 wm.obj_import
            try:
                bpy.ops.wm.obj_import(filepath=mesh_path)
            except Exception:
                try:
                    bpy.ops.import_scene.obj(filepath=mesh_path)
                except Exception as e:
                    raise RuntimeError(f"OBJ 导入失败: {e}")
        elif ext in ('.dae', '.xml'):
            # 尝试常见的 Collada 导入操作名
            try:
                bpy.ops.wm.collada_import(filepath=mesh_path)
            except Exception:
                try:
                    bpy.ops.import_scene.collada(filepath=mesh_path)
                except Exception:
                    try:
                        bpy.ops.import_scene.dae(filepath=mesh_path)
                    except Exception as e:
                        raise RuntimeError(f"Collada 导入失败: {e}")
        else:
            # 回退策略：先尝试 STL（Blender 4.1+ 新 API），再 OBJ，再 Collada
            tried = False
            try:
                bpy.ops.wm.stl_import(filepath=mesh_path)
                tried = True
            except Exception:
                try:
                    bpy.ops.import_mesh.stl(filepath=mesh_path)
                    tried = True
                except Exception:
                    pass
            if not tried:
                try:
                    bpy.ops.wm.obj_import(filepath=mesh_path)
                    tried = True
                except Exception:
                    try:
                        bpy.ops.import_scene.obj(filepath=mesh_path)
                        tried = True
                    except Exception:
                        pass
            if not tried:
                try:
                    bpy.ops.wm.collada_import(filepath=mesh_path)
                except Exception as e:
                    raise RuntimeError(f"找不到合适的导入器来导入: {mesh_path} \n{e}")
    except Exception as e:
        raise RuntimeError(f"导入网格时出错: {mesh_path} -> {e}")

    # 导入后新建对象
    post_objs = set(bpy.context.scene.objects)
    new_objs = [o for o in post_objs - pre_objs]

    # 提取 mesh 元素的 scale 属性（如果存在）
    scale_attr = mesh.attrib.get('scale')
    if scale_attr:
        try:
            scale_vals = [float(s) for s in scale_attr.split()]
            scale = Vector(scale_vals) if len(scale_vals) == 3 else None
        except Exception:
            scale = None
    else:
        scale = None

    # 对导入的对象应用缩放（从 mesh 属性或 transform_apply）并删除相机/灯光
    cams_lights = [o for o in new_objs if o.type in ('CAMERA', 'LIGHT')]
    for obj in new_objs:
        if obj.type != 'CAMERA' and obj.type != 'LIGHT':
            select_only(obj)
            # 先应用 scale 属性（如果有）
            if scale is not None:
                obj.scale = scale
            try:
                bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
            except Exception:
                pass

    if cams_lights:
        bpy.ops.object.select_all(action='DESELECT')
        for o in cams_lights:
            o.select_set(True)
        bpy.ops.object.delete()

    # 返回导入的非相机/灯光对象，保持与原插件行为兼容
    imported_mesh_objects = [o for o in new_objs if o.type not in ('CAMERA', 'LIGHT')]
    return imported_mesh_objects


def load_geometry(visual):
    geometry = visual.find('geometry')

    mesh = geometry.find('mesh')
    if mesh is not None:
        return load_mesh(mesh)

    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        length = float(cylinder.attrib['length'])
        radius = float(cylinder.attrib['radius'])
        bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=radius, depth=length)
        return [bpy.context.active_object]

    box = geometry.find('box')
    if box is not None:
        x, y, z = [float(s) for s in box.attrib['size'].split()]
        bpy.ops.mesh.primitive_cube_add()
        cube = bpy.context.active_object
        cube.dimensions = Vector((x, y, z))
        bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
        return [cube]

    sphere = geometry.find('sphere')
    if sphere is not None:
        radius = float(sphere.attrib['radius'])
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=3, radius=radius)
        return [bpy.context.active_object]
        
    return []


def get_world_min_z(obj):
    """Return the lowest world-space Z value of an object's bounding box."""
    return min((obj.matrix_world @ Vector(corner)).z for corner in obj.bound_box)


def align_robot_to_ground(armature, ground_z=0.0):
    """Move the imported robot so its lowest mesh point sits on ground_z."""
    robot_objects = [
        obj for obj in bpy.data.objects
        if obj == armature or obj.name.startswith('TF_') or obj.name.startswith('DEFORM__')
    ]
    mesh_objects = [obj for obj in robot_objects if obj.type == 'MESH']

    if not mesh_objects:
        return 0.0

    min_z = min(get_world_min_z(obj) for obj in mesh_objects)
    offset_z = ground_z - min_z

    # 只有在模型最低点低于地面时才上移，避免破坏本来就正确的模型位置
    if offset_z <= 0.0:
        return 0.0

    for obj in robot_objects:
        obj.location.z += offset_z

    bpy.context.view_layer.update()
    return offset_z


def add_revolute_joint_bone(armature, joint, empty, parent_bone_name):
    axis = Vector([float(s) for s in joint.find('axis').attrib['xyz'].split()])
    axis_world = empty.matrix_world.to_3x3() @ axis

    select_only(armature)
    bpy.ops.object.mode_set(mode='EDIT')
    eb = armature.data.edit_bones.new(joint.attrib['name'])
    eb.head = empty.location
    eb.tail = empty.location + axis_world / 10
    eb.parent = armature.data.edit_bones[parent_bone_name]
    bone_name = eb.name

    bpy.ops.object.mode_set(mode='POSE')

    posebone = armature.pose.bones[bone_name]

    posebone.rotation_mode = 'XYZ'
    posebone.lock_rotation[0] = True
    posebone.lock_rotation[1] = False
    posebone.lock_rotation[2] = True

    posebone.lock_ik_x = True
    posebone.lock_ik_y = False
    posebone.lock_ik_z = True

    bpy.ops.object.mode_set(mode='OBJECT')
    return bone_name


def position_link_objects(visual, objects, empty, joint_name):
    for i, obj in enumerate(objects):
        if obj is None:
            continue
        select_only(obj)
        print(bpy.context.selected_objects)
        obj.name = 'DEFORM__' + joint_name + '__' + str(i)
        
        # 设置对象的基础位置和旋转为 empty 的值（保留缩放）
        obj.location = empty.location
        obj.rotation_euler = empty.rotation_euler
        
        # 应用 visual 中的 origin 变换
        origin = visual.find('origin')
        if origin is not None:
            if 'xyz' in origin.attrib:
                translation = [float(s) for s in origin.attrib['xyz'].split()]
                obj.location = obj.location + Vector(translation)
            
            if 'rpy' in origin.attrib:
                roll, pitch, yaw = [float(s) for s in origin.attrib['rpy'].split()]
                # 应用欧拉角旋转
                from mathutils import Euler
                euler = Euler((roll, pitch, yaw), 'XYZ')
                obj.rotation_euler = euler
            
        bpy.context.view_layer.update()


def add_childjoints(armature, joints, links, link, empty, parent_bone_name):
    childjoints = find_childjoints(joints, link)
    for childjoint in childjoints:
        new_empty = add_next_empty(empty, childjoint)
        
        bone_name = parent_bone_name
        
        if childjoint.attrib['type'] == 'revolute':
            bone_name = add_revolute_joint_bone(armature, childjoint, new_empty, parent_bone_name)
            
        # Find the childlink xml object
        childlink_name = childjoint.find('child').attrib['link']
        for childlink in links:
            if childlink.attrib['name'] == childlink_name:
                break
        
        # 处理所有 visual 元素
        visuals = childlink.findall('visual')
        for visual in visuals:
            objects = load_geometry(visual)
            position_link_objects(visual, objects, new_empty, bone_name)
        
        add_childjoints(armature, joints, links, childlink_name, new_empty, bone_name)


def assign_vertices_to_group(object, groupname):
    select_only(object)
    group = object.vertex_groups[groupname]
    indices = [v.index for v in bpy.context.selected_objects[0].data.vertices]
    group.add(indices, 1.0, type='ADD')


def import_urdf(filepath):
    if not os.path.exists(filepath):
        print('File does not exist')

    # 保存用户原始 3D 游标状态，导入结束后恢复
    original_cursor_location = bpy.context.scene.cursor.location.copy()
    original_cursor_rotation = bpy.context.scene.cursor.rotation_euler.copy()

    # 设置 URDF 根目录，供解析相对 mesh 路径使用
    global URDF_DIR
    URDF_DIR = os.path.dirname(os.path.abspath(filepath))

    tree = ET.parse(filepath)
    xml_root = tree.getroot()

    links = xml_root.findall('link')
    joints = xml_root.findall('joint')

    if joints:
        rootlinks = find_rootlinks(joints)
    else:
        rootlinks = [link.attrib['name'] for link in links]

    for rootlink in rootlinks:
        bpy.context.scene.cursor.location = Vector((0.0, 0.0, 0.0))
        bpy.context.scene.cursor.rotation_euler = Vector((0.0, 0.0, 0.0))

        bpy.ops.object.empty_add(type='ARROWS', align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))
        bpy.context.object.empty_display_size = 0.2
        empty = bpy.context.active_object
        empty.name = 'TF_' + rootlink
        
        bpy.ops.object.armature_add(radius=0.05, enter_editmode=False, align='WORLD', location=(0, 0, 0), scale=(1, 1, 1))    
        armature = bpy.context.active_object

        bone_name = 'root'
        bpy.context.active_bone.name = bone_name

        select_only(armature)
        bpy.ops.object.mode_set(mode='POSE')
        armature.pose.bones[bone_name].lock_ik_x = True
        armature.pose.bones[bone_name].lock_ik_y = True
        armature.pose.bones[bone_name].lock_ik_z = True
        bpy.ops.object.mode_set(mode='OBJECT')

    
        for link in links:
            if link.attrib['name'] == rootlink:
                break

        # 处理所有 visual 元素，而不只是第一个
        visuals = link.findall('visual')
        for visual in visuals:
            objects = load_geometry(visual)
            for i, obj in enumerate(objects):
                if obj and hasattr(obj, 'type') and obj.type == 'MESH':
                    # 设置rootlink网格的名称和位置
                    obj.name = 'DEFORM__root__' + str(i)
                    origin = visual.find('origin')
                    if origin is not None:
                        if 'xyz' in origin.attrib:
                            translation = [float(s) for s in origin.attrib['xyz'].split()]
                            obj.location = obj.location + Vector(translation)
                        if 'rpy' in origin.attrib:
                            roll, pitch, yaw = [float(s) for s in origin.attrib['rpy'].split()]
                            from mathutils import Euler
                            euler = Euler((roll, pitch, yaw), 'XYZ')
                            obj.rotation_euler = euler

        
        add_childjoints(armature, joints, links, rootlink, empty, bone_name)

        # 将机器人整体抬到地面，避免根坐标位于躯干中部时下半身穿地
        align_robot_to_ground(armature)

        ## Skinning
        select_only(armature)

        for obj in bpy.data.objects:
            if 'DEFORM__' in obj.name:
                obj.select_set(True)

        bpy.ops.object.parent_set(type='ARMATURE_NAME')

        for object in bpy.data.objects:
            if 'DEFORM__' in object.name:
                groupname = object.name.split('__')[1]
                assign_vertices_to_group(object, groupname)

        # Delete the empties
        bpy.ops.object.select_all(action='DESELECT') 
        for object in bpy.data.objects:
            if 'TF_' in object.name:
                object.select_set(True)
        bpy.ops.object.delete() 
        select_only(armature)

    # 恢复导入前 3D 游标状态，避免游标停留在关节/脚底位置
    bpy.context.scene.cursor.location = original_cursor_location
    bpy.context.scene.cursor.rotation_euler = original_cursor_rotation


if __name__ == '__main__':
    filepath = '/home/idlab185/ur10.urdf'
    import_urdf(filepath)
