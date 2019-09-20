import os, glob
import numpy as np
import vtk
import math
import classes
# intrinsic calibration
class renderer:
    @classmethod
    def draw_camera(cls, vtk_renderer, camera, color_str):
        t = camera.SE3[0:3, 3].reshape(3,)
        R = camera.SE3[0:3, 0:3].reshape(3, 3)
        wh = camera.sensor_size[0] * 0.5
        hh = camera.sensor_size[1] * 0.5
        focal_length = (camera.focal_lengths[0]+camera.focal_lengths[1]) / 2.

        size_scaler = 20.
        wh *= size_scaler
        hh *= size_scaler
        focal_length *= size_scaler

        p0 = np.array([-wh, hh, focal_length])
        p1 = np.array([wh, hh, focal_length])
        p2 = np.array([wh, -hh, focal_length])
        p3 = np.array([-wh, -hh, focal_length])
        p4 = np.array([0, 0, 0])

        p0 = R.dot(p0) + t
        p1 = R.dot(p1) + t
        p2 = R.dot(p2) + t
        p3 = R.dot(p3) + t
        p4 = R.dot(p4) + t

        points = vtk.vtkPoints()
        points.InsertNextPoint(p0)
        points.InsertNextPoint(p1)
        points.InsertNextPoint(p2)
        points.InsertNextPoint(p3)
        points.InsertNextPoint(p4)

        # camera face1
        tri1 = vtk.vtkTriangle()
        tri1.GetPointIds().SetId(0, 0)
        tri1.GetPointIds().SetId(1, 1)
        tri1.GetPointIds().SetId(2, 2)

        # camera face2
        tri2 = vtk.vtkTriangle()
        tri2.GetPointIds().SetId(0, 0)
        tri2.GetPointIds().SetId(1, 2)
        tri2.GetPointIds().SetId(2, 3)

        # camera side right
        tri3 = vtk.vtkTriangle()
        tri3.GetPointIds().SetId(0, 0)
        tri3.GetPointIds().SetId(1, 4)
        tri3.GetPointIds().SetId(2, 3)
        # camera side left
        tri4 = vtk.vtkTriangle()
        tri4.GetPointIds().SetId(0, 1)
        tri4.GetPointIds().SetId(1, 4)
        tri4.GetPointIds().SetId(2, 2)
        # camera side top
        tri5 = vtk.vtkTriangle()
        tri5.GetPointIds().SetId(0, 0)
        tri5.GetPointIds().SetId(1, 1)
        tri5.GetPointIds().SetId(2, 4)
        # camera side bottom
        tri6 = vtk.vtkTriangle()
        tri6.GetPointIds().SetId(0, 2)
        tri6.GetPointIds().SetId(1, 3)
        tri6.GetPointIds().SetId(2, 4)

        triangles = vtk.vtkCellArray()
        triangles.InsertNextCell(tri1)
        triangles.InsertNextCell(tri2)
        triangles.InsertNextCell(tri3)
        triangles.InsertNextCell(tri4)
        triangles.InsertNextCell(tri5)
        triangles.InsertNextCell(tri6)

        # setup colors (setting the name to "Colors" is nice but not necessary)
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        colors.SetName("Colors")
        namedColors = vtk.vtkNamedColors()
        try:
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color_str))
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color_str))
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color_str))
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color_str))
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color_str))
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color_str))
        except AttributeError:
            # For compatibility with new VTK generic data arrays.
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color_str))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color_str))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color_str))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color_str))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color_str))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color_str))

        # polydata object
        trianglePolyData = vtk.vtkPolyData()
        trianglePolyData.SetPoints(points)
        trianglePolyData.SetPolys(triangles)
        trianglePolyData.GetCellData().SetScalars(colors)

        # mapper
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(trianglePolyData)
        else:
            mapper.SetInputData(trianglePolyData)

        # actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # assign actor to the renderer
        vtk_renderer.AddActor(actor)
        renderer.draw_axis(vtk_renderer, R, t, 2, 500)

        # draw ray in +z direction of camera
        # ray_length = 3000
        # r = np.array([0, 0, ray_length])
        # r = R.dot(r) + t
        # renderer.draw_ray(vtk_renderer, p4, r, 1, 'Blue')
        return actor
    @classmethod
    def draw_floor(cls, vtk_renderer, w, h, color):
        p0 = np.array([-w, h, 0])
        p1 = np.array([w, h, 0])
        p2 = np.array([w, -h, 0])
        p3 = np.array([-w, -h, 0])

        points = vtk.vtkPoints()
        points.InsertNextPoint(p0)
        points.InsertNextPoint(p1)
        points.InsertNextPoint(p2)
        points.InsertNextPoint(p3)

        # checkboard triangle 1
        tri1 = vtk.vtkTriangle()
        tri1.GetPointIds().SetId(0, 0)
        tri1.GetPointIds().SetId(1, 1)
        tri1.GetPointIds().SetId(2, 2)
        tri2 = vtk.vtkTriangle()
        tri2.GetPointIds().SetId(0, 0)
        tri2.GetPointIds().SetId(1, 2)
        tri2.GetPointIds().SetId(2, 3)

        triangles = vtk.vtkCellArray()
        triangles.InsertNextCell(tri1)
        triangles.InsertNextCell(tri2)

        # polydata object
        trianglePolyData = vtk.vtkPolyData()
        trianglePolyData.SetPoints(points)
        trianglePolyData.SetPolys(triangles)

        # Create a vtkUnsignedCharArray container and store the colors in it
        namedColors = vtk.vtkNamedColors()
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        try:
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color))
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color))
        except AttributeError:
            # For compatibility with new VTK generic data arrays.
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color))
        trianglePolyData.GetCellData().SetScalars(colors)

        # mapper
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(trianglePolyData)
        else:
            mapper.SetInputData(trianglePolyData)

        # actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetPosition(0, 0, 0)
        vtk_renderer.AddActor(actor)
        return actor




    @classmethod
    def edit_ray_mapper(cls, vtk_actor, p0s, p1s, width, colors_in):
        linesPolyData = vtk.vtkPolyData()
        # Create a vtkCellArray container and store the lines in it
        lines = vtk.vtkCellArray()
        for i in range(len(p0s)):
            pts = vtk.vtkPoints()
            pts.InsertNextPoint(p0s[i])
            pts.InsertNextPoint(p1s[i])

            # Add the points to the polydata container
            linesPolyData.SetPoints(pts)

            # Create the line between p0 and p1
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, 0)
            line.GetPointIds().SetId(1, 1)
            lines.InsertNextCell(line)

        # Add the lines to the polydata container
        linesPolyData.SetLines(lines)

        # Create a vtkUnsignedCharArray container and store the colors in it
        namedColors = vtk.vtkNamedColors()
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        try:
            for i in range(len(colors_in)):
                colors.InsertNextTupleValue(namedColors.GetColor3ub(colors_in[i]))
        except AttributeError:
            for i in range(len(colors_in)):
                # For compatibility with new VTK generic data arrays.
                colors.InsertNextTypedTuple(namedColors.GetColor3ub(colors_in[i]))
        linesPolyData.GetCellData().SetScalars(colors)

        # Setup the visualization pipeline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(linesPolyData)
        vtk_actor.GetProperty().SetLineWidth(width)
        vtk_actor.SetMapper(mapper)

    @classmethod
    def draw_ray(cls, vtk_renderer, p0, p1, width, color):
        # Create the polydata where we will store all the geometric data
        linesPolyData = vtk.vtkPolyData()
        pts = vtk.vtkPoints()
        pts.InsertNextPoint(p0)
        pts.InsertNextPoint(p1)

        # Add the points to the polydata container
        linesPolyData.SetPoints(pts)

        # Create the line between p0 and p1
        line = vtk.vtkLine()
        line.GetPointIds().SetId(0, 0)
        line.GetPointIds().SetId(1, 1)

        # Create a vtkCellArray container and store the lines in it
        lines = vtk.vtkCellArray()
        lines.InsertNextCell(line)

        # Add the lines to the polydata container
        linesPolyData.SetLines(lines)

        # Create a vtkUnsignedCharArray container and store the colors in it
        namedColors = vtk.vtkNamedColors()
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        try:
            colors.InsertNextTupleValue(namedColors.GetColor3ub(color))
        except AttributeError:
            # For compatibility with new VTK generic data arrays.
            colors.InsertNextTypedTuple(namedColors.GetColor3ub(color))
        linesPolyData.GetCellData().SetScalars(colors)

        # Setup the visualization pipeline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(linesPolyData)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetLineWidth(width)

        vtk_renderer.AddActor(actor)
        return actor

    @classmethod
    def draw_checkboard(cls, vtk_renderer, chb):
        w = chb.dim[0]
        h = chb.dim[1]

        p0 = np.array([0, 0, 0])
        p1 = np.array([-w, 0, 0])
        p2 = np.array([-w, h, 0])
        p3 = np.array([0, h, 0])

        R = chb.SE3[0:3, 0:3].reshape(3, 3)
        t = chb.SE3[0:3, 3].reshape(3, )
        #
        # p0 = R.dot(p0) + t
        # p1 = R.dot(p1) + t
        # p2 = R.dot(p2) + t
        # p3 = R.dot(p3) + t

        points = vtk.vtkPoints()
        points.InsertNextPoint(p0)
        points.InsertNextPoint(p1)
        points.InsertNextPoint(p2)
        points.InsertNextPoint(p3)

        # checkboard triangle 1
        tri1 = vtk.vtkTriangle()
        tri1.GetPointIds().SetId(0, 0)
        tri1.GetPointIds().SetId(1, 1)
        tri1.GetPointIds().SetId(2, 2)
        tri2 = vtk.vtkTriangle()
        tri2.GetPointIds().SetId(0, 0)
        tri2.GetPointIds().SetId(1, 2)
        tri2.GetPointIds().SetId(2, 3)

        triangles = vtk.vtkCellArray()
        triangles.InsertNextCell(tri1)
        triangles.InsertNextCell(tri2)

        # polydata object
        trianglePolyData = vtk.vtkPolyData()
        trianglePolyData.SetPoints(points)
        trianglePolyData.SetPolys(triangles)

        # Create a vtkUnsignedCharArray container and store the colors in it
        namedColors = vtk.vtkNamedColors()
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        try:
            colors.InsertNextTupleValue(namedColors.GetColor3ub('Black'))
            colors.InsertNextTupleValue(namedColors.GetColor3ub('Black'))
        except AttributeError:
            # For compatibility with new VTK generic data arrays.
            colors.InsertNextTypedTuple(namedColors.GetColor3ub('Black'))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub('Black'))
        trianglePolyData.GetCellData().SetScalars(colors)

        # mapper
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(trianglePolyData)
        else:
            mapper.SetInputData(trianglePolyData)

        # actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetPosition(t[0], t[1], t[2])

        r = tool.rodrgiue([0, 1, 0], np.pi/2)
        axis, ang = tool.Rot2Axisangle(r)
        actor.RotateWXYZ(ang, axis[0], axis[1], axis[2])
        # assign actor to the renderer
        vtk_renderer.AddActor(actor)

        renderer.draw_axis(vtk_renderer, R, t, 2, 10)
        return actor
    @classmethod
    def draw_axis(cls, vtk_renderer, R, t, width, length):
        x = length*np.array([1, 0, 0])
        y = length*np.array([0, 1, 0])
        z = length*np.array([0, 0, 1])

        x = R.dot(x) + t
        y = R.dot(y) + t
        z = R.dot(z) + t

        # Create the polydata where we will store all the geometric data
        linesPolyData = vtk.vtkPolyData()
        # Create a vtkPoints container and store the points in it
        pts = vtk.vtkPoints()
        pts.InsertNextPoint(t)
        pts.InsertNextPoint(x)
        pts.InsertNextPoint(y)
        pts.InsertNextPoint(z)

        # Add the points to the polydata container
        linesPolyData.SetPoints(pts)

        # Create the first line (between Origin and x)
        line_x = vtk.vtkLine()
        line_x.GetPointIds().SetId(0, 0)  # the second 0 is the index of the Origin in linesPolyData's points
        line_x.GetPointIds().SetId(1, 1)  # the second 1 is the index of x in linesPolyData's points

        # Create the second line (between Origin and y)
        line_y = vtk.vtkLine()
        line_y.GetPointIds().SetId(0, 0)  # the second 0 is the index of the Origin in linesPolyData's points
        line_y.GetPointIds().SetId(1, 2)  # 2 is the index of y in linesPolyData's points

        # Create the second line (between Origin and z)
        line_z = vtk.vtkLine()
        line_z.GetPointIds().SetId(0, 0)  # the second 0 is the index of the Origin in linesPolyData's points
        line_z.GetPointIds().SetId(1, 3)  # 2 is the index of z in linesPolyData's points

        # Create a vtkCellArray container and store the lines in it
        lines = vtk.vtkCellArray()
        lines.InsertNextCell(line_x)
        lines.InsertNextCell(line_y)
        lines.InsertNextCell(line_z)

        # Add the lines to the polydata container
        linesPolyData.SetLines(lines)

        namedColors = vtk.vtkNamedColors()

        # Create a vtkUnsignedCharArray container and store the colors in it
        colors = vtk.vtkUnsignedCharArray()
        colors.SetNumberOfComponents(3)
        try:
            colors.InsertNextTupleValue(namedColors.GetColor3ub("Red"))
            colors.InsertNextTupleValue(namedColors.GetColor3ub("Green"))
            colors.InsertNextTupleValue(namedColors.GetColor3ub("Blue"))
        except AttributeError:
            # For compatibility with new VTK generic data arrays.
            colors.InsertNextTypedTuple(namedColors.GetColor3ub("Red"))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub("Green"))
            colors.InsertNextTypedTuple(namedColors.GetColor3ub("Blue"))

        # Color the lines.
        # SetScalars() automatically associates the values in the data array passed as parameter
        # to the elements in the same indices of the cell data array on which it is called.
        # This means the first component (red) of the colors array
        # is matched with the first component of the cell array (line 0)
        # and the second component (green) of the colors array
        # is matched with the second component of the cell array (line 1)
        linesPolyData.GetCellData().SetScalars(colors)

        # Setup the visualization pipeline
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(linesPolyData)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetLineWidth(width)

        vtk_renderer.AddActor(actor)
        return actor

    @classmethod
    def render_scene(cls, vtk_renderer, cameras):
        # cameras
        colors = ['Red', 'Orange', 'Yellow', 'Green', 'Blue', 'Navy', 'Purple', 'Cyan']
        cam_actors = []
        for camera in cameras:
            if camera.is_available:
                actor = cls.draw_camera(vtk_renderer, camera, colors[camera.index])
                if camera.render:
                    actor.GetProperty().SetOpacity(1.0)
                else:
                    actor.GetProperty().SetOpacity(0.1)

                cam_actors.append(actor)
        # floor
        floor_w = 3500
        floor_h = 2500
        color = 'azure'
        cls.draw_floor(vtk_renderer, floor_w, floor_h, color)

        # global axis
        axis_length = 1000 # 1m
        cls.draw_axis(vtk_renderer, np.identity(3), [0, 0, 0], 3, axis_length)
        vtk_renderer.SetBackground(1, 1, 1)
        return cam_actors
    @classmethod
    def draw_3d_point(cls, vtk_renderer, point, radius, color):
        # create source
        source = vtk.vtkSphereSource()
        source.SetCenter(0, 0, 0)
        source.SetRadius(radius)

        # mapper
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(source.GetOutput())
        else:
            mapper.SetInputConnection(source.GetOutputPort())

        # actor
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(color[0], color[1], color[2])
        actor.SetPosition(point[0], point[1], point[2])
        # assign actor to the renderer
        vtk_renderer.AddActor(actor)

        return actor


class tool:
    @classmethod
    def fetch_files_with_extension(cls, path, ext, order=1):
        # order=1: increasing, -1: deceasing
        os.chdir(path)
        fetched_list = sorted(glob.glob("*." + ext))
        f0 = os.path.splitext(fetched_list[0])[0]
        f1 = os.path.splitext(fetched_list[1])[0]
        if (order == -1 and int(f0) < int(f1)) or (order == 1 and int(f0) > int(f1)):
            fetched_list.reverse()
        return fetched_list

    @classmethod
    def rodrgiue(cls, axis, rad):
        X = tool.skew_sym(axis)
        R = np.identity(3) + np.sin(rad)*(X) + (1.0 - np.cos(rad))*X.dot(X)
        return R.astype('float32')

    @classmethod
    def skew_sym(cls, axis):
        x = axis[0]
        y = axis[1]
        z = axis[2]
        return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]]).astype('float32')
    @classmethod
    def unskew_sym(cls, V):
        v = np.array([V[2,1]-V[1, 2], V[0,2]-V[2,0], V[1,0]-V[0,1]]).astype('float32')
        return v
    @classmethod
    def to_SE3(cls, R, t):
        H = np.array([R[0,0], R[0,1], R[0,2], t[0], R[1,0], R[1,1], R[1,2], t[1], R[2,0], R[2,1], R[2,2], t[2], 0, 0, 0, 1]).reshape((4, 4))
        return H.astype('float32')

    @classmethod
    def to_R_t(cls, H):
        R = np.array([H[0,0], H[0,1], H[0,2], H[1,0], H[1,1], H[1,2], H[2,0], H[2,1], H[2,2]]).reshape((3, 3))
        t = np.array([H[0,3], H[1,3], H[2,3]]).reshape((3, ))
        return R.astype('float32'), t.astype('float32')

    @classmethod
    def inv_SE3(cls, H):
        R, t = tool.to_R_t(H)
        Rinv = R.T
        tinv = R.T.dot(-t)
        return tool.to_SE3(Rinv, tinv)

    @classmethod
    def R2Quat(cls, R):
        qw = np.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2.
        qx = (R[2,1] - R[1,2]) / (4 * qw)
        qy = (R[0,2] - R[2,0]) / (4 * qw)
        qz = (R[1,0] - R[0,1]) / (4 * qw)
        return [qx, qy, qz, qw]

    @classmethod
    def Rot2Axisangle(cls, R):
        # R is SO(3), 3x3 numpy matrix
        u_skew_sym = R-R.transpose()

        # IF R IS IDENTITY, u IS ZERO VECTOR
        u = cls.unskew_sym(u_skew_sym)

        if np.linalg.norm(u) < 0.0001:
            print('[WARNING] Rot2Axisangle -> R is almost identity')
        u = u / np.linalg.norm(u)
        rad = math.acos((R.trace()-1.0)/2.0)
        return u, rad