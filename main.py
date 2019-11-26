import vtk

def ReadPolyData(file_name):
    import os
    path, extension = os.path.splitext(file_name)
    extension = extension.lower()
    if extension == ".ply":
        reader = vtk.vtkPLYReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == ".vtp":
        reader = vtk.vtkXMLpoly_dataReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == ".obj":
        reader = vtk.vtkOBJReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == ".stl":
        reader = vtk.vtkSTLReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == ".vtk":
        reader = vtk.vtkpoly_dataReader()
        reader.SetFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    elif extension == ".g":
        reader = vtk.vtkBYUReader()
        reader.SetGeometryFileName(file_name)
        reader.Update()
        poly_data = reader.GetOutput()
    else:
        # Return a None if the extension is unknown.
        poly_data = None
    return poly_data


def render_poly_data(polydata):
    # Setup actor and mapper
    mapper = vtk.vtkPolyDataMapper()
    if vtk.VTK_MAJOR_VERSION <= 5:
        mapper.SetInputConnection(polydata.GetProducerPort())
    else:
        mapper.SetInputData(polydata)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetPointSize(10)
    # Setup render window, renderer, and interactor
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)
    renderer.AddActor(actor)
    renderWindow.Render()
    renderWindowInteractor.Start()


def generate_colored_polydata(polydata):
    bounds = polydata.GetBounds()
    minz = bounds[-2]
    maxz = bounds[-1]
    # Create the color map
    colorLookupTable = vtk.vtkLookupTable()
    colorLookupTable.SetTableRange(minz, maxz)
    colorLookupTable.Build()
    # Generate the colors for each point based on the color map
    colors = vtk.vtkUnsignedCharArray()
    colors.SetNumberOfComponents(3)
    colors.SetName("Colors")
    print("There are " + str(polydata.GetNumberOfPoints()) + " points.")
    for i in range(0, polydata.GetNumberOfPoints()):
        p = 3 * [0.0]
        polydata.GetPoint(i, p)

        dcolor = 3 * [0.0]
        colorLookupTable.GetColor(p[2], dcolor);
        color = 3 * [0.0]
        for j in range(0, 3):
            color[j] = int(255.0 * dcolor[j])

        try:
            colors.InsertNextTupleValue(color)
        except AttributeError:
            # For compatibility with new VTK generic data arrays.
            colors.InsertNextTypedTuple(color)
    polydata.GetPointData().SetScalars(colors)


polydata = ReadPolyData("assets/10680_Dog_v2.obj")

generate_colored_polydata(polydata)

render_poly_data(polydata)
