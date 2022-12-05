# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

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
    pcd = o3d.io.read_point_cloud("/home/libish/libish6.ply")

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
    o3d.io.write_point_cloud("outlier_libish6.ply", outlier_cloud)
    
    outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
    outlier_cloud.orient_normals_towards_camera_location(camera_location=[0, 0, 0])
    outlier_cloud.orient_normals_consistent_tangent_plane(30)   

    hull = ConvexHull(outlier_cloud, qhull_options="QJ")
    # o3d.visualization.draw_geometries([outlier_cloud]) 

    # #Get centroid
    cx = np.mean(hull.points[hull.vertices,0])
    cy = np.mean(hull.points[hull.vertices,1])
    cz = np.mean(hull.points[hull.vertices,2])


    axis_aligned_bounding_box = outlier_cloud.get_axis_aligned_bounding_box()
    axis_aligned_bounding_box.color = (1, 0, 0)
    oriented_bounding_box = outlier_cloud.get_oriented_bounding_box()
    oriented_bounding_box.color = (0, 1, 0)
    print(
        "Displaying axis_aligned_bounding_box in red and oriented bounding box in green ..."
    )
    o3d.visualization.draw(
        [outlier_cloud, axis_aligned_bounding_box, oriented_bounding_box])

    #Plot convex hull
    for simplex in hull.simplices:
     points = labels
    plt.plot(points[simplex, 0], points[simplex, 1], 'k-')

    #Plot centroid
    plt.plot(cx, cy, cz,'x',ms=20)
    plt.show()
