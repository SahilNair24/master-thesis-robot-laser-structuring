import pyvista as pv
import numpy as np

# Define your target position in robot space
# target_position = np.array([0.5, 0.2, 0.8])

# Load a CAD model
mesh = pv.read("/home/sahilsnair/Desktop/laser-toolpath/scripts/cad_models/FemaleHead.obj")
# mesh = pv.read("/home/sahilsnair/Desktop/laser-toolpath/scripts/cad_models/new/curv_obj_1m.stl")
mesh = mesh.clip('z', invert=False,value=5)


# print("Old center", center)
# #Define your target position in robot space
# target_position = np.array([0.0, 0.0, 0.0])  # meters, for example
# # Compute the translation vector
# # translation_vector = target_position - center
# translation_vector = target_position-center
# mesh = mesh.translate(translation_vector, inplace=False)
# mesh.save("clipped_FemaleHead.stl")
# center_new = mesh.center
# print("Center_new:", center_new)

# plotter = pv.Plotter(off_screen=True)
plotter = pv.Plotter()
plotter.add_mesh(mesh, color="lightblue", show_edges=True)
# plotter.screenshot("images/head_original.png")
# plotter.add_axes_at_origin()
plotter.show()
