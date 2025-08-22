'''
 * Plugin Name: MeLiDA - Measurement LiDAR system
 * Plugin URI:  https://github.com/MeLiDAsys/MeLiDA
 * Description: A field-proven, building-scale 2D LiDAR measurement system.
 * Version:     0.1.0-beta
 * Author:      Maciej Płotkowiak
 * Author URI:  https://github.com/MeLiDAsys/MeLiDA
 * License:     GNU AGPLv3
'''

import bpy
from bpy.app.handlers import persistent
from bpy.props import BoolProperty
from bpy.types import AddonPreferences,Operator
import struct,serial,math,time,gpu
from gpu_extras.batch import batch_for_shader
import mathutils
current_mesh=None
USBuffer=None
BlendScale=.01
PointRad=1.
liveOnOff=False
oneModAtTime=None
timer1=None
drawPoint=None
CPRenderDATA=False
point_groups={}
def setup_serial_connection():
	try:USBuffer=serial.Serial('COM3',230400,timeout=5);return USBuffer
	except serial.SerialException as e:print(f"Serial Port error: {e}");return
def lidar_reading_function(USBuffer):
	retries=2
	for _ in range(retries):
		try:
			data_rip=synchronize_and_read_packet(USBuffer);points=parse_packet(data_rip)
			if points:cartPoints,pointsQa=update_points(points);return cartPoints,pointsQa
			time.sleep(1)
		except serial.SerialException as e:print(f"Data read error: {e}");break
	return[],[]
def synchronize_and_read_packet(USBuffer):USBuffer.timeout=.5;data_rip=USBuffer.read_all();USBuffer.reset_input_buffer();return data_rip
def parse_packet(data_rip):
	points=[]
	for i in range(len(data_rip)-47):
		if data_rip[i]==84 and data_rip[i+1]==44:Sangle,*Pdata,Eangle=struct.unpack('<H'+'HB'*12+'H',data_rip[i+4:i+44]);points.append((Sangle,Eangle,Pdata))
	return points
def update_points(points):
	global BlendScale;cartPoints=[];pointsQa=[]
	for i in range(len(points)):
		Sangle=points[i][0];Eangle=points[i][1]
		if Eangle<Sangle:Eangle+=36000
		step=(Eangle-Sangle)/11
		for j in range(0,len(points[i][2]),2):
			pointDs=points[i][2][j];pointQa=points[i][2][j+1]
			if 10<pointDs<1000 or pointQa<bpy.context.scene.my_tool.pointQaLimit:continue
			pointEn=math.radians((Sangle+step*(j/2))/1e2);skale=BlendScale/10;x=pointDs*math.sin(pointEn)*skale;y=pointDs*math.cos(pointEn)*skale;z=0;pointsQa.append(pointQa);cartPoints.append((x,y,z))
	return cartPoints,pointsQa
class PointDrawer:
	def add_point_group(group_id,coords,intesiti):global point_groups;point_groups[group_id]=coords,intesiti
	def update_point_group(group_id):
		global point_groups,CPRenderDATA
		if group_id in point_groups:obj=bpy.data.objects.get(group_id);CPRenderDATA=True;points=[(obj.matrix_world@v.co).to_tuple()for v in obj.data.vertices];old_intensity=point_groups[group_id][1];point_groups[group_id]=points,old_intensity;CPRenderDATA=False
		else:print('No group with that name')
class liveViewClass(bpy.types.Operator):
	bl_idname='wm.liveview';bl_label='Live View'
	def execute(self,context):
		global liveOnOff,USBuffer,timer1,drawPoint;liveOnOff=not liveOnOff
		if liveOnOff:
			USBuffer=setup_serial_connection()
			if USBuffer:self.vertices=[];timer1=context.window_manager.event_timer_add(.1,window=context.window);drawPoint=bpy.types.SpaceView3D.draw_handler_add(self.draw_callback,(context,),'WINDOW','POST_VIEW');context.window_manager.modal_handler_add(self);return{'RUNNING_MODAL'}
			else:print("Can't connect to LIDAR head");return self.cancel(context)
		else:return self.cancel(context)
	def modal(self,context,event):
		global liveOnOff
		if event.type=='TIMER':cartPoints,pointsQa=lidar_reading_function(USBuffer);self.vertices=[mathutils.Vector((x,y,z))for(x,y,z)in cartPoints];context.area.tag_redraw()
		if not liveOnOff:return{'FINISHED'}
		return{'PASS_THROUGH'}
	def draw_callback(self,context):shader=gpu.shader.from_builtin('UNIFORM_COLOR');batch=batch_for_shader(shader,'POINTS',{'pos':self.vertices});gpu.state.point_size_set(3);shader.bind();shader.uniform_float('color',(1.,.0,.0,1.));batch.draw(shader)
	def cancel(self,context):
		global USBuffer,timer1,drawPoint,liveOnOff;liveOnOff=False;context.window_manager.event_timer_remove(timer1);timer1=None;bpy.types.SpaceView3D.draw_handler_remove(drawPoint,'WINDOW');drawPoint=None
		if USBuffer:USBuffer.close();USBuffer=None
		return{'CANCELLED'}
class MakePointsClass(bpy.types.Operator):
	bl_idname='wm.makepoints';bl_label='Make Points'
	@classmethod
	def poll(cls,context):global liveOnOff;return liveOnOff is False
	def invoke(self,context,event):global oneModAtTime,CPRenderDATA;oneModAtTime=True;CPRenderDATA=True;return self.execute(context)
	def execute(self,context):
		global USBuffer,oneModAtTime,CPRenderDATA
		def makepoints(cartPoints,pointsQa):
			coords=[];intesiti=[];existing_numbers={int(c.name.split('_')[-1])for c in bpy.data.meshes if c.name.startswith('PointCloud_')and c.name.split('_')[-1].isdigit()};next_number=1
			while next_number in existing_numbers:next_number+=1
			mesh_name=f"PointCloud_{next_number}";mesh=bpy.data.meshes.new(mesh_name);mesh['tag']='MeLiDA_PC';obj=bpy.data.objects.new(mesh_name,mesh);bpy.context.collection.objects.link(obj);mesh.from_pydata(cartPoints,[],[]);color_attribute=mesh.color_attributes.new(name='Color',type='BYTE_COLOR',domain='POINT')
			for(i,color)in enumerate(pointsQa):color=color/255;color_attribute.data[i].color=1.-color,color,.0,1.;intesiti.append((1.-color,color,.0,1.))
			mesh.update();PointDrawer.add_point_group(mesh_name,cartPoints,intesiti)
		if not USBuffer:USBuffer=setup_serial_connection()
		cartPoints,pointsQa=lidar_reading_function(USBuffer)
		if cartPoints and pointsQa:
			makepoints(cartPoints,pointsQa)
			if USBuffer:USBuffer.close();USBuffer=None
		oneModAtTime=False;CPRenderDATA=False;return{'FINISHED'}
class CPcolorRender(bpy.types.Operator):
	bl_idname='wm.cpcolorrender';bl_label='Intesiti point color for PC'
	def execute(self,context):global drawPoint;drawPoint=bpy.types.SpaceView3D.draw_handler_add(self.draw_callback,(context,),'WINDOW','POST_VIEW');return{'FINISHED'}
	def draw_callback(self,context):
		global CPRenderDATA,point_groups
		if CPRenderDATA is False:
			if CPRenderDATA is False:
				for(group_id,(coords,intesiti))in point_groups.items():shader=gpu.shader.from_builtin('FLAT_COLOR');gpu.state.point_size_set(3);batch=batch_for_shader(shader,'POINTS',{'pos':coords,'color':intesiti});shader.bind();batch.draw(shader)
class CPcolorLoad(bpy.types.Operator):
	bl_idname='wm.cpcolorload';bl_label=''
	def execute(self,context):
		global drawPoint,point_groups;coords=[]
		for obj in bpy.context.view_layer.objects:
			if obj.name.startswith('PointCloud_'):
				if obj.type=='MESH':
					mesh=obj.data;coords_global=[obj.matrix_world@v.co for v in mesh.vertices];coords=[(v.x,v.y,v.z)for v in coords_global];color_attribute=mesh.color_attributes.get('Color')
					if color_attribute and color_attribute.data_type=='BYTE_COLOR':intesiti=[(color.color[0],color.color[1],color.color[2])for color in color_attribute.data];point_groups[obj.name]=coords,intesiti
		return{'FINISHED'}
class CPrenderONOFF(bpy.types.Operator):
	bl_idname='wm.cprenderonoff';bl_label='CP render ON OFF'
	def execute(self,context):
		global CPRenderDATA;CPRenderDATA=not CPRenderDATA
		for area in bpy.context.screen.areas:
			if area.type=='VIEW_3D':area.tag_redraw()
		return{'FINISHED'}
class pointQaLimit_prompt(bpy.types.PropertyGroup):pointQaLimit:bpy.props.IntProperty(name='Quality limit',description='0-255',default=0,min=0,max=255)
class IterfacePanel(bpy.types.Panel):
	bl_label='MeLiDA';bl_idname='MELIDA_PT_Iterface';bl_space_type='VIEW_3D';bl_region_type='UI';bl_category='MeLiDA'
	def draw(self,context):layout=self.layout;scene=context.scene;layout.operator('wm.liveview',text='Live View');layout.prop(scene.my_tool,'pointQaLimit');layout.operator('wm.makepoints',text='Make Points');layout.operator('wm.cprenderonoff',text='CP render ON OFF')
def track_object_move(scene,depsgraph):
	for update in depsgraph.updates:
		if update.is_updated_transform:
			if isinstance(update.id,bpy.types.Object):
				obj=update.id
				if obj.name.startswith('PointCloud_'):PointDrawer.update_point_group(obj.name)
def on_object_remove(scene,depsgraph):
	delgrup=[]
	for name in point_groups:
		if name not in bpy.context.view_layer.objects:delgrup.append(name)
	for name in delgrup:del point_groups[name]
def startupBackgroundF(*args):bpy.ops.wm.cpcolorrender();bpy.app.handlers.depsgraph_update_post.append(track_object_move);bpy.app.handlers.depsgraph_update_post.append(on_object_remove);bpy.app.handlers.load_post.remove(startupBackgroundF)
@persistent
def loadFileF(dummyArg):
	bpy.ops.wm.cpcolorload()
	if track_object_move not in bpy.app.handlers.depsgraph_update_post:bpy.app.handlers.depsgraph_update_post.append(track_object_move)
	if on_object_remove not in bpy.app.handlers.depsgraph_update_post:bpy.app.handlers.depsgraph_update_post.append(on_object_remove)
class WelcomeMessageAddonPreferences(AddonPreferences):
	bl_idname=__name__;has_shown_welcome:BoolProperty(name='Welcome Shown',default=False)
	def draw(self,context):layout=self.layout
class WELCOME_OT_DialogOperator(Operator):
	bl_idname='welcome.dialog_operator';bl_label='Welcome Message';bl_options={'INTERNAL'}
	def execute(self,context):return{'FINISHED'}
	def invoke(self,context,event):wm=context.window_manager;return context.window_manager.invoke_confirm(self,event,title='MeLiDA:',message="Hi! I'm moving into your Blender ^^. Give me second: RESTART Blender pleassse!",confirm_text='OK',icon='WARNING')
	def draw(self,context):layout=self.layout;layout.label(text='Thank you for installing this add-on!');layout.label(text='This message will only appear once.')
def check_welcome_message():
	try:
		prefs=bpy.context.preferences.addons[__name__].preferences
		if prefs is None:return .5
		if not prefs.has_shown_welcome:bpy.ops.welcome.dialog_operator('INVOKE_DEFAULT');prefs.has_shown_welcome=True;bpy.ops.wm.save_userpref()
	except(KeyError,AttributeError):return .5
def register():
	bpy.utils.register_class(WelcomeMessageAddonPreferences);bpy.utils.register_class(WELCOME_OT_DialogOperator);bpy.app.timers.register(check_welcome_message,first_interval=1.);bpy.utils.register_class(CPcolorRender);bpy.utils.register_class(CPcolorLoad);bpy.utils.register_class(liveViewClass);bpy.utils.register_class(MakePointsClass);bpy.utils.register_class(CPrenderONOFF);bpy.utils.register_class(IterfacePanel);bpy.utils.register_class(pointQaLimit_prompt);bpy.types.Scene.my_tool=bpy.props.PointerProperty(type=pointQaLimit_prompt);bpy.types.Scene.liveOnOff=bpy.props.BoolProperty(name='Live View Toggle',description='Toggle for Live View',default=False);wm=bpy.context.window_manager;kc=wm.keyconfigs.addon
	if kc:km=kc.keymaps.new(name='3D View',space_type='VIEW_3D');km.keymap_items.new('view3d.rotatet','MIDDLEMOUSE','PRESS');km.keymap_items.new('view3d.scrollt','WHEELUPMOUSE','PRESS');km.keymap_items.new('view3d.scrollt','WHEELDOWNMOUSE','PRESS');km.keymap_items.new('object.select_by_tag','LEFTMOUSE','CLICK',shift=False,ctrl=False,alt=False)
	bpy.app.handlers.load_post.append(startupBackgroundF);bpy.app.handlers.load_post.append(loadFileF)
def unregister():bpy.utils.unregister_class(WelcomeMessageAddonPreferences);bpy.utils.unregister_class(WELCOME_OT_DialogOperator);bpy.utils.unregister_class(CPcolorRender);bpy.utils.unregister_class(CPcolorLoad);bpy.utils.unregister_class(liveViewClass);bpy.utils.unregister_class(MakePointsClass);bpy.utils.unregister_class(CPrenderONOFF);bpy.utils.unregister_class(pointQaLimit_prompt);del bpy.types.Scene.my_tool;bpy.utils.unregister_class(IterfacePanel);del bpy.types.Scene.liveOnOff;bpy.app.handlers.load_post.remove(loadFileF)