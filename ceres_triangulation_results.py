import calib_single
import calib_stereo
import functions
import classes
import numpy as np
import cv2
import random
import vtk

window_w = 4500
window_h = 3000

cam_actors = []
world_point_actors = []
curr_frame_idx = 1
max_frame_idx = 9999
window = vtk.vtkRenderWindow()
vtk_renderer = vtk.vtkRenderer()
text_actor = vtk.vtkTextActor()
txtprop = text_actor.GetTextProperty()
txtprop.SetFontFamilyToArial()
txtprop.SetFontSize(30)
txtprop.SetColor(0, 0, 0)
text_actor.SetDisplayPosition(30, 30)
vtk_renderer.AddActor(text_actor)
ray_actors = []

def keypress_callback(obj, ev):
    global curr_frame_idx
    key = obj.GetKeySym()
    if key == 'Right':
        curr_frame_idx = min(curr_frame_idx + 1, max_frame_idx)
        render_frame(curr_frame_idx)
    elif key == 'Left':
        curr_frame_idx = max(curr_frame_idx - 1, 1)
        render_frame(curr_frame_idx)
    elif key == 'Up':
        curr_frame_idx = min(curr_frame_idx + 100, max_frame_idx)
        render_frame(curr_frame_idx)
    elif key =='Down':
        curr_frame_idx = max(curr_frame_idx - 100, 1)
        render_frame(curr_frame_idx)

def render_frame(frame_idx):
    global vtk_renderer, cam_actors, cameras, world_point_actors, world_points, text_actor

    # update camera
    colors = ['Red', 'Green', 'Blue']
    for camera in cameras:
        chb_detected = chb_detects[camera.index][frame_idx]
        if chb_detected:
            cam_actors[camera.index].GetProperty().SetOpacity(1.0)

            # unproject measurement points to 3d as a ray
            # p0s = []
            # p1s = []
            # for pi in range(len(camera.measured_2d_points[frame_idx])):
            #     uv = camera.measured_2d_points[frame_idx][pi]
            #     ray_start, d_ray = camera.unproject_2d_to_3d(uv)
            #     lambda_scalar = 3000
            #     p0s.append(ray_start)
            #     p1s.append(ray_start + lambda_scalar*d_ray)
            # functions.renderer.edit_ray_mapper(ray_actors[camera.index], p0s, p1s, 1, colors)
        else:
            cam_actors[camera.index].GetProperty().SetOpacity(0.1)


    # update 3d points
    p1 = world_points[0][frame_idx, :]
    p2 = world_points[1][frame_idx, :]
    p3 = world_points[2][frame_idx, :]
    world_point_actors[0].SetPosition(p1[0], p1[1], p1[2])
    world_point_actors[1].SetPosition(p2[0], p2[1], p2[2])
    world_point_actors[2].SetPosition(p3[0], p3[1], p3[2])

    # get orientation of checkboard
    v1 = np.array(p2-p1)
    v2 = np.array(p3-p1)
    p4 = p1 + v1 + v2
    world_point_actors[3].SetPosition(p4[0], p4[1], p4[2])
    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)
    v3 = np.cross(v2, v1)
    v2 = np.cross(v1, v3)


    # create a text actor
    img_name = int(img_names[frame_idx])
    text_actor.SetInput(str(img_name)+".pgm")

    window.Render()

cameras = [classes.Camera(0, r'\A001_05290330_C001', True), classes.Camera(1, r'\B001_07072140_C001', True),
           classes.Camera(2, r'\C001_06070320_C001', True), classes.Camera(3, r'\D001_05061515_C001', True),
           classes.Camera(4, r'\E001_04210958_C001', True), classes.Camera(5, r'\F001_05151133_C001', True),
           classes.Camera(6, r'\G001_05052127_C001', True), classes.Camera(7, r'\H001_05191504_C001', True)]

calib_single.load_intrinscs_extrinsics_as_world_origin_is_center(cameras)
for camera in cameras:
    r_actor = functions.renderer.draw_ray(vtk_renderer, [0, 0, 0], [0, 0, 0], 1, 'k')
    ray_actors.append(r_actor)

# load ceres result data
tri_result_path = r'C:\Users\joont\Documents\Visual Studio 2017\Projects\CeresTest\CeresTest\world_points_est.txt'
img_names, cAs, cBs, cCs, cDs, cEs, cFs, cGs, cHs, x1s, y1s, z1s, x2s, y2s, z2s, x3s, y3s, z3s = zip(*((map(float, line.split()) for line in open(tri_result_path, 'r'))))
chb_detects = [cAs, cBs, cCs, cDs, cEs, cFs, cGs, cHs]
max_frame_idx = len(img_names)-1

world_point1 = np.array([x1s, y1s, z1s]).transpose()
world_point2 = np.array([x2s, y2s, z2s]).transpose()
world_point3 = np.array([x3s, y3s, z3s]).transpose()
world_points = [world_point1, world_point2, world_point3]

# load 2d image points used for triangulation
img_pt_path = r'C:\Users\joont\Documents\Visual Studio 2017\Projects\CeresTest\DataSet\_dataset_triangulation.txt'
with open(img_pt_path, 'r') as f:
    for line_idx, line in enumerate(f):
        if line_idx == 0:
            continue
        vs = line.split()
        for c in range(len(cameras)):
            uv1 = np.array([vs[3 + 7 * c + 0], vs[3 + 7 * c + 1]])
            uv2 = np.array([vs[3 + 7 * c + 2], vs[3 + 7 * c + 3]])
            uv3 = np.array([vs[3 + 7 * c + 4], vs[3 + 7 * c + 5]])
            cameras[c].measured_2d_points.append([uv1, uv2, uv3])
    f.close()

# rendering pipeline
win_size_scaler = 0.25
window.SetSize(int(win_size_scaler*classes.Camera.image_res[0]), int(win_size_scaler*classes.Camera.image_res[1]))
interactor = vtk.vtkRenderWindowInteractor()
interactor.AddObserver('KeyPressEvent', keypress_callback, 1.0)
interactor.SetRenderWindow(window)

# Define viewport ranges
window.AddRenderer(vtk_renderer)

# draw scene
cam_actors = functions.renderer.render_scene(vtk_renderer, cameras)

# draw world points, set opacity to 0 for now
radius = 50
world_point_actor1 = functions.renderer.draw_3d_point(vtk_renderer, world_points[0][0, :], radius, [1, 0, 0])
world_point_actor2 = functions.renderer.draw_3d_point(vtk_renderer, world_points[1][0, :], radius, [0, 1, 0])
world_point_actor3 = functions.renderer.draw_3d_point(vtk_renderer, world_points[2][0, :], radius, [0, 0, 1])
world_point_actor4 = functions.renderer.draw_3d_point(vtk_renderer, world_points[2][0, :], radius, [1, 1, 1])
world_point_actors.append(world_point_actor1)
world_point_actors.append(world_point_actor2)
world_point_actors.append(world_point_actor3)
world_point_actors.append(world_point_actor4)

# set camera positions
up_vec = np.array([0, 0, 1])
vtk_renderer.ResetCamera()
vtk_camera = vtk.vtkCamera()
vtk_camera.SetPosition(0, window_h, window_w)
vtk_camera.SetFocalPoint(0, 0, 0)
vtk_camera.SetViewUp(up_vec[0], up_vec[1], up_vec[2])
# vtk_camera.OrthogonalizeViewUp()
vtk_renderer.SetActiveCamera(vtk_camera)

interactor.Initialize()
window.Render()
interactor.Start()

print('here')
