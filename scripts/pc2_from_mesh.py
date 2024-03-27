import numpy as np
import open3d as o3d
import open3d.visualization.rendering as rendering
import trimesh as tm
OBJ_PATH = "PepsiCan/Pepsi_Can2.obj"
SAVE_PATH = "Pepsi_Can.ply"
def get_pc_from_mesh():
    # tm_mesh = tm.load("PepsiCan/Pepsi_Can.obj")
    # # tm_mesh.show()
    # tm_mesh.export("PepsiCan/Pepsi_Can2.obj")

    mesh = o3d.io.read_triangle_mesh(OBJ_PATH)
    mesh.compute_vertex_normals()
    # material = rendering.MaterialRecord()
    # material.shader = 'defaultUnlit'
    # material.albedo_img = o3d.io.read_image('PepsiCan/PepsiTexture.jpg')

    
    # visualize the mesh with colors
    o3d.visualization.draw_geometries([mesh])
    # o3d.visualization.draw({'name': 'box', 'geometry': mesh, 'material': material})

    # extract every point on the surface of the mesh
    pcd = mesh.sample_points_uniformly(number_of_points=40000)
    # visualize the point cloud
    o3d.visualization.draw_geometries([pcd])
    # save the point cloud
    o3d.io.write_point_cloud(SAVE_PATH, pcd)

if __name__ == "__main__":
    get_pc_from_mesh()
    # pcd = o3d.io.read_point_cloud(SAVE_PATH)


