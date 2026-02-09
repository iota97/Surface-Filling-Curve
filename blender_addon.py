bl_info = {
    "name": "Surface Filling Curve",
    "blender": (4, 5, 0),
    "category": "Object",
}

import bpy, tempfile, os, subprocess, threading

class SufaceFillingCurveAddonPreferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    exe_path: bpy.props.StringProperty(
        name="Executable Path",
        subtype='FILE_PATH',
    )

    def draw(self, context):
        layout = self.layout
        layout.prop(self, "exe_path")

class surfaceFillingCurveGeometryNode():
    group = None

    @staticmethod
    def getGroup(force = False):
        if not surfaceFillingCurveGeometryNode.group or force:
            surfaceFillingCurveGeometryNode.group = surfaceFillingCurveGeometryNode.generate()    
        return surfaceFillingCurveGeometryNode.group
    
    @staticmethod
    def generate():
        surface_filling_curve_solid = bpy.data.node_groups.new(type = 'GeometryNodeTree', name = "Surface Filling Curve Solid")
        surface_filling_curve_solid.is_modifier = True
        geometry_socket = surface_filling_curve_solid.interface.new_socket(name = "Geometry", in_out='OUTPUT', socket_type = 'NodeSocketGeometry')
        geometry_socket.attribute_domain = 'POINT'
        geometry_socket_1 = surface_filling_curve_solid.interface.new_socket(name = "Geometry", in_out='INPUT', socket_type = 'NodeSocketGeometry')
        geometry_socket_1.attribute_domain = 'POINT'
        radius_socket = surface_filling_curve_solid.interface.new_socket(name = "Radius", in_out='INPUT', socket_type = 'NodeSocketFloat')
        radius_socket.default_value = 1.0
        radius_socket.min_value = 0.0
        radius_socket.max_value = 3.4028234663852886e+38
        radius_socket.subtype = 'DISTANCE'
        radius_socket.attribute_domain = 'POINT'
        scale_socket = surface_filling_curve_solid.interface.new_socket(name = "Anisotropy", in_out='INPUT', socket_type = 'NodeSocketFloat')
        scale_socket.default_value = 0.5
        scale_socket.min_value = 0.0
        scale_socket.max_value = 3.4028234663852886e+38
        scale_socket.subtype = 'NONE'
        scale_socket.attribute_domain = 'POINT'
        group_input = surface_filling_curve_solid.nodes.new("NodeGroupInput")
        group_input.name = "Group Input"
        group_output = surface_filling_curve_solid.nodes.new("NodeGroupOutput")
        group_output.name = "Group Output"
        group_output.is_active_output = True
        scale_elements = surface_filling_curve_solid.nodes.new("GeometryNodeScaleElements")
        scale_elements.name = "Scale Elements"
        scale_elements.domain = 'EDGE'
        scale_elements.scale_mode = 'SINGLE_AXIS'
        scale_elements.inputs[1].default_value = True
        scale_elements.inputs[3].default_value = (0.0, 0.0, 0.0)
        scale_elements.inputs[4].default_value = (1.0, 0.0, 0.0)
        mesh_circle = surface_filling_curve_solid.nodes.new("GeometryNodeMeshCircle")
        mesh_circle.name = "Mesh Circle"
        mesh_circle.fill_type = 'NONE'
        mesh_circle.inputs[0].default_value = 32
        mesh_to_curve = surface_filling_curve_solid.nodes.new("GeometryNodeMeshToCurve")
        mesh_to_curve.name = "Mesh to Curve"
        mesh_to_curve.inputs[1].default_value = True
        curve_to_mesh = surface_filling_curve_solid.nodes.new("GeometryNodeCurveToMesh")
        curve_to_mesh.name = "Curve to Mesh"
        curve_to_mesh.inputs[2].default_value = True
        mesh_to_curve_001 = surface_filling_curve_solid.nodes.new("GeometryNodeMeshToCurve")
        mesh_to_curve_001.name = "Mesh to Curve.001"
        mesh_to_curve_001.inputs[1].default_value = True
        resample_curve = surface_filling_curve_solid.nodes.new("GeometryNodeResampleCurve")
        resample_curve.name = "Resample Curve"
        resample_curve.mode = 'LENGTH'
        resample_curve.inputs[1].default_value = True
        set_curve_normal = surface_filling_curve_solid.nodes.new("GeometryNodeSetCurveNormal")
        set_curve_normal.name = "Set Curve Normal"
        set_curve_normal.mode = 'FREE'
        named_attribute = surface_filling_curve_solid.nodes.new("GeometryNodeInputNamedAttribute")
        named_attribute.name = "Named Attribute"
        named_attribute.data_type = 'FLOAT_VECTOR'
        named_attribute.inputs[0].default_value = "normal"
        group_input.location = (-353.3892822265625, 240.4407958984375)
        group_output.location = (660.0, 280.0)
        scale_elements.location = (86.97605895996094, 251.82894897460938)
        mesh_circle.location = (-111.18216705322266, 241.97567749023438)
        mesh_to_curve.location = (-117.0793228149414, 384.1482849121094)
        curve_to_mesh.location = (492.32879638671875, 300.4315490722656)
        mesh_to_curve_001.location = (284.1856384277344, 254.6247100830078)
        resample_curve.location = (283.7376403808594, 445.71856689453125)
        set_curve_normal.location = (98.99124145507812, 498.23895263671875)
        named_attribute.location = (-113.77352142333984, 524.2737426757812)
        group_input.width, group_input.height = 140.0, 100.0
        group_output.width, group_output.height = 140.0, 100.0
        scale_elements.width, scale_elements.height = 140.0, 100.0
        mesh_circle.width, mesh_circle.height = 140.0, 100.0
        mesh_to_curve.width, mesh_to_curve.height = 140.0, 100.0
        curve_to_mesh.width, curve_to_mesh.height = 140.0, 100.0
        mesh_to_curve_001.width, mesh_to_curve_001.height = 140.0, 100.0
        resample_curve.width, resample_curve.height = 140.0, 100.0
        set_curve_normal.width, set_curve_normal.height = 140.0, 100.0
        named_attribute.width, named_attribute.height = 140.0, 100.0
        surface_filling_curve_solid.links.new(group_input.outputs[1], mesh_circle.inputs[1])
        surface_filling_curve_solid.links.new(mesh_circle.outputs[0], scale_elements.inputs[0])
        surface_filling_curve_solid.links.new(curve_to_mesh.outputs[0], group_output.inputs[0])
        surface_filling_curve_solid.links.new(group_input.outputs[0], mesh_to_curve.inputs[0])
        surface_filling_curve_solid.links.new(scale_elements.outputs[0], mesh_to_curve_001.inputs[0])
        surface_filling_curve_solid.links.new(mesh_to_curve_001.outputs[0], curve_to_mesh.inputs[1])
        surface_filling_curve_solid.links.new(mesh_to_curve.outputs[0], set_curve_normal.inputs[0])
        surface_filling_curve_solid.links.new(named_attribute.outputs[0], set_curve_normal.inputs[2])
        surface_filling_curve_solid.links.new(named_attribute.outputs[1], set_curve_normal.inputs[1])
        surface_filling_curve_solid.links.new(set_curve_normal.outputs[0], resample_curve.inputs[0])
        surface_filling_curve_solid.links.new(resample_curve.outputs[0], curve_to_mesh.inputs[0])
        surface_filling_curve_solid.links.new(group_input.outputs[1], resample_curve.inputs[3])
        surface_filling_curve_solid.links.new(group_input.outputs[2], scale_elements.inputs[2])
        return surface_filling_curve_solid
    
class SufaceFillingCurveOperator(bpy.types.Operator):
    """Surface Filling Curve"""
    bl_idname = "object.surface_filling_curve"
    bl_label = "Surface Filling Curve"
    bl_options = {'REGISTER'}
    
    timer = None
    thread = None
    log = []
    mutex = threading.Lock()
    obj = None
    running = False
    handle = None
    killed = False
    output_log = []
    progress = [0.0]
    
    @staticmethod
    def kill():
        if SufaceFillingCurveOperator.handle:
            SufaceFillingCurveOperator.killed = True
            SufaceFillingCurveOperator.handle.terminate()
            SufaceFillingCurveOperator.handle.wait()
            SufaceFillingCurveOperator.handle = None

    def modal(self, context, event):
        if event.type == 'TIMER':
            self.output_log.clear()
            self.progress[0] = 0.0
            with self.mutex:
                for i in self.log:
                    self.output_log.append(i)
                    self.progress[0] += 1.0/11.0 if "ms" in i else 0

            for a in bpy.context.screen.areas:
                a.tag_redraw()
            if not self.thread.is_alive():
                SufaceFillingCurveOperator.running = False
                self.thread.join()
                if SufaceFillingCurveOperator.killed:
                    SufaceFillingCurveOperator.killed = False
                else:
                    self.after(context)
                return {'FINISHED'}
        return {'PASS_THROUGH'}
    
    def cancel(self, context):
        context.window_manager.event_timer_remove(self.timer)
        
    def run(self, cmd): 
        for line in self.run_cmd(cmd):
            with self.mutex:
                if line[:len('Progress')] == 'Progress':
                    self.log[-1] = line[:line.index('%')+1]
                else:
                    self.log.append(line) 
                    
    def run_cmd(self, cmd):
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE, universal_newlines=True)
        SufaceFillingCurveOperator.handle = popen
        for stdout_line in iter(popen.stdout.readline, ""):
            yield stdout_line 
        popen.stdout.close()
        return_code = popen.wait()
        if return_code:
            raise subprocess.CalledProcessError(return_code, cmd)

    def execute(self, context):
        if SufaceFillingCurveOperator.running:
            return {'CANCELLED'}
        SufaceFillingCurveOperator.running = True
        if not self.before(context):
            SufaceFillingCurveOperator.running = False
            return {'CANCELLED'}
        with self.mutex:
            self.log.clear()   
        cmd = self.generate_cmd(context)
        if not os.path.isfile(cmd[0]):
            SufaceFillingCurveOperator.running = False
            return {'CANCELLED'}
        
        self.thread = threading.Thread(target=self.run, args=(cmd,))
        self.thread.start()
        wm = context.window_manager
        self.timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}
    
    def before(self, context):
        mesh_path = tempfile.gettempdir() + '/Curve.ply'
        self.obj = context.active_object
        try:
            old_mode = bpy.context.object.mode
        except:
            return False
        bpy.ops.object.mode_set(mode='OBJECT')
        try:
            bpy.ops.wm.ply_export(filepath=mesh_path, export_uv=False, export_triangulated_mesh=True,\
                ascii_format=True, check_existing=False, export_selected_objects=True)
        except:
            pass

        bpy.ops.object.mode_set(mode=old_mode)
        return True
    
    def generate_cmd(self, context):
        mesh_path = tempfile.gettempdir() + '/Curve.ply'
        exe_path = bpy.path.abspath(context.preferences.addons[__name__].preferences.exe_path)
        cmd = [ exe_path, mesh_path, str(bpy.context.window_manager.surface_curve_width),  "-c", mesh_path ]
        if bpy.context.window_manager.surface_curve_resample:
            cmd.append("-r")
        if bpy.context.window_manager.surface_curve_repulse:
            cmd.append("-R")
        cmd.append(bpy.context.window_manager.surface_curve_intrinsic)
        return cmd
    
    def after(self, context):
        mesh_path = tempfile.gettempdir() + '/Curve.ply'
        try:
            old_mode = bpy.context.object.mode
            bpy.ops.object.mode_set(mode='OBJECT')
        except:
            old_mode = None
        if bpy.context.window_manager.surface_curve_replace: 
            try:
                bpy.data.objects.remove(bpy.context.window_manager.surface_curve_added, do_unlink=True)
            except:
                pass
        try:
            bpy.ops.wm.ply_import(filepath=mesh_path)
            bpy.context.window_manager.surface_curve_added = context.active_object
            if bpy.context.window_manager.surface_curve_solid:
                try:
                    modifier = bpy.context.active_object.modifiers.new(name='Solid', type='NODES')
                    try:
                        modifier.node_group = surfaceFillingCurveGeometryNode.getGroup()
                    except:
                        modifier.node_group = surfaceFillingCurveGeometryNode.getGroup(True)
                    modifier['Socket_2'] = 0.005*bpy.context.window_manager.surface_curve_width*bpy.context.window_manager.surface_curve_fill
                    modifier['Socket_3'] = bpy.context.window_manager.surface_curve_anisotropy
                    bpy.ops.object.modifier_apply(modifier=modifier.name)
                except:
                    pass   
                
            bpy.context.view_layer.objects.active = bpy.data.objects[self.obj.name]
            bpy.ops.object.select_all(action='DESELECT')
            bpy.data.objects[self.obj.name].select_set(True)
            bpy.context.view_layer.objects.active = bpy.data.objects[self.obj.name]
        except:
            pass
        try:
            bpy.ops.object.mode_set(mode=old_mode)
        except:
            pass

class SufaceFillingCurveOperatorCancel(bpy.types.Operator):
    """Surface Filling Curve Cancel"""
    bl_idname = "object.surface_filling_curve_cancel"
    bl_label = "Surface Filling Curve Cancel"
    bl_options = {'REGISTER'}
    
    def execute(self, context):
        SufaceFillingCurveOperator.kill()
        return {'CANCELLED'}
    
class SufaceFillingCurvePanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Curve"
    bl_idname = "filling_curve"
    bl_label = "Surface Filling Curve"

    def draw(self, context):
        layout = self.layout
        col = layout.column()
        col.prop(context.window_manager, "surface_curve_width", text="Spacing")
        col.separator()
        col.prop(context.window_manager, "surface_curve_intrinsic", text="Directions")
        col.prop(context.window_manager, "surface_curve_resample", text="Resample")
        col.prop(context.window_manager, "surface_curve_repulse", text="Repulse [Experimental]")
        if bpy.app.version < (5, 0, 0):
            col.prop(context.window_manager, "surface_curve_solid", text="Solid")
            if context.window_manager.surface_curve_solid:
                col.prop(context.window_manager, "surface_curve_fill", text="Fill Percentage")
                col.prop(context.window_manager, "surface_curve_anisotropy", text="Aspect Ratio")
        col.prop(context.window_manager, "surface_curve_replace", text="Replace Last")
        col.separator()
        if SufaceFillingCurveOperator.running:
            col.progress(text="        Generating...", factor=SufaceFillingCurveOperator.progress[0])
            col.operator(SufaceFillingCurveOperatorCancel.bl_idname, text="Cancel", icon="X")
        else:
            col.operator(SufaceFillingCurveOperator.bl_idname, text="Generate", icon="CONSOLE", depress=SufaceFillingCurveOperator.running)
        col.prop(context.window_manager, "surface_curve_show", text="Show Log")
        if context.window_manager.surface_curve_show:
            for line in SufaceFillingCurveOperator.output_log:
                col.row().label(text=line)

def register():
    bpy.utils.register_class(SufaceFillingCurveOperator)
    bpy.utils.register_class(SufaceFillingCurveOperatorCancel)
    bpy.utils.register_class(SufaceFillingCurvePanel)
    bpy.utils.register_class(SufaceFillingCurveAddonPreferences)
    bpy.types.WindowManager.surface_curve_width = bpy.props.FloatProperty(default=1.0, min=1e-8)
    bpy.types.WindowManager.surface_curve_resample = bpy.props.BoolProperty(default=False)
    bpy.types.WindowManager.surface_curve_repulse = bpy.props.BoolProperty(default=False)
    bpy.types.WindowManager.surface_curve_replace = bpy.props.BoolProperty(default=True)
    bpy.types.WindowManager.surface_curve_intrinsic = bpy.props.EnumProperty(items=[ ('-i', "Intrinsic", ""),('-e', "Extrinsic", ""),('-p', "Parallel", ""),('-n', "Nearest", ""),('-l', "3D Printing", ""),])
    bpy.types.WindowManager.surface_curve_solid = bpy.props.BoolProperty(default=False)
    bpy.types.WindowManager.surface_curve_added = bpy.props.PointerProperty(type=bpy.types.ID)
    bpy.types.WindowManager.surface_curve_fill = bpy.props.FloatProperty(default=30.0, min=1e-8)
    bpy.types.WindowManager.surface_curve_show = bpy.props.BoolProperty(default=False)
    bpy.types.WindowManager.surface_curve_anisotropy = bpy.props.FloatProperty(default=1.0, min=0)

def unregister():
    bpy.utils.unregister_class(SufaceFillingCurvePanel)
    bpy.utils.unregister_class(SufaceFillingCurveOperatorCancel)
    bpy.utils.unregister_class(SufaceFillingCurveOperator)
    bpy.utils.unregister_class(SufaceFillingCurveAddonPreferences)
    del bpy.types.WindowManager.surface_curve_width
    del bpy.types.WindowManager.surface_curve_resample
    del bpy.types.WindowManager.surface_curve_repulse
    del bpy.types.WindowManager.surface_curve_replace
    del bpy.types.WindowManager.surface_curve_intrinsic
    del bpy.types.WindowManager.surface_curve_solid
    del bpy.types.WindowManager.surface_curve_added
    del bpy.types.WindowManager.surface_curve_fill
    del bpy.types.WindowManager.surface_curve_show
    del bpy.types.WindowManager.surface_curve_anisotropy

if __name__ == "__main__":
    register()