import numpy as np
import open3d as o3d

print("Load a ply point cloud, print it, and render it")
pcd = o3d.read_point_cloud("H:\Contests_Geospatial\Contest 01 Data\Cloud.ply")
print(pcd)
print(np.asarray(pcd.points))
o3d.visualization.draw_geometries([pcd])

print("Recompute the normal of the downsampled point cloud")
o3d.geometry.estimate_normals(
        pcd,
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                          max_nn=30))
o3d.visualization.draw_geometries([pcd])

[m, c]= o3d.geometry.compute_point_cloud_mean_and_covariance(pcd)
print(m)

print(c)
