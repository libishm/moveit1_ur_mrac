import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

if __name__ == "__main__":

    # mesh = o3d.io.read_triangle_mesh("/home/libish/scanned mesh/15.ply")
    # print(mesh)
    # o3d.visualization.draw_geometries([mesh])
    # #simplify mesh      
    # cleanmesh = mesh.simplify_quadric_decimation(20000)
    # pcd = cleanmesh.sample_points_poisson_disk(number_of_points=10000, init_factor=20)
    # ply_point_cloud = o3d.data.PLYPointCloud()
    # pcd = o3d.io.read_point_cloud("/home/libish/scanned mesh/15.ply")
    pcd = o3d.io.read_point_cloud("/home/libish/libish8.ply")

    # Flip it, otherwise the pointcloud will be upside down.
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw([inlier_cloud, outlier_cloud])
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            outlier_cloud.cluster_dbscan(eps=0.025, min_points=50, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw([outlier_cloud])
    o3d.io.write_point_cloud("outlier_libish8.ply", outlier_cloud)
    
    outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
    outlier_cloud.orient_normals_towards_camera_location(camera_location=[0, 0, 0])
    outlier_cloud.orient_normals_consistent_tangent_plane(30)

    hull = ConvexHull(outlier_cloud, qhull_options="QJ")
    o3d.visualization.draw_geometries([outlier_cloud]) 

    # #Get centroid
    cx = np.mean(hull.points[hull.vertices,0])
    cy = np.mean(hull.points[hull.vertices,1])
    cz = np.mean(hull.points[hull.vertices,2])

    #Plot convex hull
    for simplex in hull.simplices:
     points = labels
    plt.plot(points[simplex, 0], points[simplex, 1], 'k-')

    #Plot centroid
    plt.plot(cx, cy, cz,'x',ms=20)
    plt.show()

