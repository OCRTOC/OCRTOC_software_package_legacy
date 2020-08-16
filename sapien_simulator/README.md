## Documentation for SAPIEN GUI
Here we talk about how to use the GUI window provided by SAPIEN.

### Introduction
The GUI SAPIEN has 2 main windows. The **Render Options** window on the left and
the **Object Properties** window on the right. It provides a way to navigate
around the scene and inspect objects.

### Navigation
To navigate around the scene, the first method is the **Free Camera**, where you
can use **WASD** keys on the keyboard to move forward, left, backward, and
right, just like in first person games. Also like in games, you can turn the
camera by holding right mouse button and move the mouse. If you are more used to
a flipped control, you can check the **Flip X** and **Flip Y** checkboxes in the
control section of the render option window.

The next method is **Focused Camera**. Activated by first left clicking an
object in the viewport, and then pressing the **F** key. In this mode, the main
camera will follow the focused object. In this mode when holding the right moues
button, instead of rotating the camera, the camera rotates around the object. In
this mode, you can also use the mouse wheel to zoom in and zoom out.

### General control
In the control section of render options. The pause checkbox will run the render
loop without advancing physics. It is good for inspecting a single frame. When
paused, the single step button shows up to break the render loop once.

Next by selecting the Transparent Selection checkbox, any object selected by
left mouse button will have transparency. This will also affect all cameras in
the scene, so should only be used when debugging.

The Gizmo checkbox will be covered later.

### Render Mode
SAPIEN supports several render modes for visualization in the Render Mode
section of Render Options. The lighting options shows normal lighted scenes. The
Albedo option shows the diffuse albedo of all objects. Normal shows normal map.
Depth shows the 0-1 exponential depth. Segmentation shows movable-part
segmentation. Custom is configured to show camera space point cloud with XYZ
encoded as RGB.

### Camera options
In camera section, you can switch to an orthographic camera view, change the FOV
or movement speed (for WASD control). The other information is probably not
useful.

### Mounted cameras
In Mounted Cameras section, you can inspect the images seen by the cameras
placed in the scene, including the Real Sense camera and the Kinect camera.

### Objects in the world
In the Object Properties window, the Global section shows pre-configured
parameters for SAPIEN, which you cannot change. In the World section, you can
see all objects and robots. The normal objects are listed under the Actors tree,
you can click on the names of the objects to select them, just like you click on
them in the viewport. For example, you can click on an object and press F. This
will focus the camera onto the object.

Robot are listed under the Articulations tree. You can further expand the robot
to see its individual links and click on the links to select them just like you
select normal objects.

For a selected object, its local frame will be shown in the viewport as axes. If
you cannot see them clearly, you can turn on the Transparent Selection option.

### Object properties
For each selected actor (normal object or robot link), you can view its property
in the Actor section and Actor Details section of the Object Properties window.
Collision shape checkbox let you see the collision body used for physical
simulation, typically larger than the visual body. The Center of Mass checkbox
will move the axes from object origin to the center of mass, with axis
representing the 3 principal components of inertia.

### Robot detail
If a selected object belongs to an articulation (robot), you can see its details
in the Articulation section. Under the Joints tree, you can see the current
joint positions in generalized coordinates. You can also drag the sliders to set
their joint position, but it is not recommended as it will break physical
simulation. By selecting details, you can see more information including joint
friction, and drive properties (which are used internally).

### Gizmo
By selecting Gizmo in Render Options > Control, or click Gizmo to Actor button
in Object Properties > Actor. You create a Gizmo controller and the Gizmo
window. Here you can drag the Gizmo controller or directly set its pose in the
Gizmo window. Next, by clicking on any dynamic actor, you get the option to
teleport it to the location of the Gizmo. The Drive Actor button will create a
spring connecting the object with the spatial pose represented by the Gizmo. The
Gizmo should only be used for debugging purposes.
