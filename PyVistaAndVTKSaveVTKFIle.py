import pyvista as pv
import vtk

def writeCorrs(scanFile, fitFile, outCorrFile, outTargetFile):
    scanData = pv.PolyData()
    jData = json.load(open(scanFile))
    scanData.points = np.array(jData["Pts"])

    fittingData = pv.PolyData(fitFile)
    goodPts = np.array(scanData.points[:,2]>0)

    numPts = scanData.points.shape[0]

    ptsVtk = vtk.vtkPoints()
    ptsAll = np.vstack([scanData.points, fittingData.points])
    # pts.InsertNextPoint(p1)
    for i in range(ptsAll.shape[0]):
        ptsVtk.InsertNextPoint(ptsAll[i, :].tolist())

    polyData = vtk.vtkPolyData()
    polyData.SetPoints(ptsVtk)

    lines = vtk.vtkCellArray()

    for i in range(goodPts.shape[0]):
        if goodPts[i]:
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, i)  # the second 0 is the index of the Origin in the vtkPoints
            line.GetPointIds().SetId(1, i + numPts)  # the second 1 is the index of P0 in the vtkPoints
            # line.
            lines.InsertNextCell(line)

    polyData.SetLines(lines)

    writer = vtk.vtkPolyDataWriter()
    writer.SetInputData(polyData)
    writer.SetFileName(outCorrFile)
    writer.Update()

    scanData.save(outTargetFile)