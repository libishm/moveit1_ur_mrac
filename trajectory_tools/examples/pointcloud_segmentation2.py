import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
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
    pcd = o3d.io.read_point_cloud("/home/libish/testy3.ply")

    # Flip it, otherwise the pointcloud will be upside down.
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=5,
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
            outlier_cloud.cluster_dbscan(eps=0.0155, min_points=150, print_progress=True))

    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    o3d.visualization.draw([outlier_cloud])
    o3d.io.write_point_cloud("outlier_highest222.ply", outlier_cloud)
    
    # outlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
    # outlier_cloud.orient_normals_towards_camera_location(camera_location=[0, 0, 0])
    # outlier_cloud.orient_normals_consistent_tangent_plane(30) 
    
    #seperate outlier_cloud into a list of numpy clusters
    clusters = []
    for i in range(max_label+1):
        clusters.append(np.asarray(outlier_cloud.points)[labels == i])
    
    #find centroid of each cluster
    centroids = []
    for i in range(max_label+1):
        centroids.append(np.mean(clusters[i], axis=0))
    centroids = np.asarray(centroids)
    print(centroids)

    centroid_cloud = o3d.geometry.PointCloud() 
    centroid_cloud.points = o3d.utility.Vector3dVector(centroids)
    o3d.visualization.draw([centroid_cloud])

    #find convex hull of each cluster
    hulls = []  
    for i in range(max_label+1):
        hulls.append(ConvexHull(clusters[i]))

    
    #find centroid of each convex hull
    hull_centroids = []
    for i in range(max_label+1):    
        hull_centroids.append(np.mean(clusters[i][hulls[i].vertices], axis=0))
    hull_centroids = np.asarray(hull_centroids)
    print(hull_centroids)

    hull_centroid_cloud = o3d.geometry.PointCloud() 
    hull_centroid_cloud.points = o3d.utility.Vector3dVector(hull_centroids)
    o3d.visualization.draw([hull_centroid_cloud])

    #find convex hull of all clusters
    all_clusters = np.concatenate(clusters, axis=0)
    all_hull = ConvexHull(all_clusters) 
    all_hull_centroid = np.mean(all_clusters[all_hull.vertices], axis=0)
    print(all_hull_centroid)

    all_hull_centroid_cloud = o3d.geometry.PointCloud()
    all_hull_centroid_cloud.points = o3d.utility.Vector3dVector(all_hull_centroid.reshape(1,3))
    o3d.visualization.draw([all_hull_centroid_cloud])

    # draw bounding box around all_hull_centroids_hull_centroid
    bounding_box = o3d.geometry.OrientedBoundingBox.create_from_points(o3d.utility.Vector3dVector(hull_centroids))
    bounding_box_cloud = bounding_box.get_box_points()

    # o3d.visualization.draw([bounding_box_cloud])
    #componentwise min and max of hull_centroids
    min_bound = np.min(hull_centroids, axis=0) 
    max_bound = np.max(hull_centroids, axis=0)
    print(min_bound, max_bound)

    o3d.geometry.crop_point_cloud(all_hull_centroid_cloud, (min_bound), (max_bound))

    #extract x and y values of hull centroids
    hull_centroids_x = hull_centroids[:,0]
    hull_centroids_y = hull_centroids[:,1]
    hull_centroids_z = hull_centroids[:,2]

    Points = np.column_stack((hull_centroids_x, hull_centroids_y, hull_centroids_z))

    #extract orientation of hull centroids
    hull_centroids_orientation = []
    for i in range(max_label+1):
     hull_centroids_orientation.append(np.asarray(outlier_cloud.normals)[labels == i][hulls[i].vertices])
    hull_centroids_orientation = np.concatenate(hull_centroids_orientation, axis=0)


    Pose = np.column_stack((hull_centroids_x, hull_centroids_y, hull_centroids_orientation[:,0], hull_centroids_orientation[:,1],))
    print(Pose)
    
    
    th.sequenncer

    
    #find the cluster with the largest z value (closest to the camera)
    max_z = 0
    index = 0
    for i in range(max_label+1):
        if hull_centroids_z[i] > max_z:
            max_z = hull_centroids_z[i]
            index = i
    #concatenate x, y, and z values of hull centroids
    position = np.column_stack((hull_centroids_x, hull_centroids_y, hull_centroids_z))
    orientation = np.column_stack((hull_centroids_orientation[:,0], hull_centroids_orientation[:,1], hull_centroids_orientation[:,2]))
    
    Pose = np.column_stack((position, orientation))
    print(Pose)


    #construct quaternion from orientation
    Quaternion = []
    for i in range(max_label+1):
        Quaternion.append(Quaternion(axis=orientation[i], angle=1))
    quaternion = np.asarray(Quaternion)
    print(quaternion)

    #find the cluster with the largest z value (closest to the camera)
    max_z = 0
    index = 0
    for i in range(max_label+1):
        if centroids[i][2] > max_z:
            max_z = centroids[i][2]
            index = i
    print(index)

    #find the cluster with the largest x value
    max_x = 0
    index_x = 0
    for i in range(max_label+1):
        if centroids[i][0] > max_x:
            max_x = centroids[i][0]
            index_x = i
    print(index_x)
    
    #find the cluster with the largest y value
    max_y = 0
    index_y = 0
    for i in range(max_label+1):
        if centroids[i][1] > max_y:
            max_y = centroids[i][1]
            index_y = i
    print(index_y)

    #find the cluster with the smallest x value
    min_x = 0
    index_min_x = 0
    for i in range(max_label+1):
        if centroids[i][0] < min_x:
            min_x = centroids[i][0]
            index_min_x = i
    print(index_min_x)

    #find the cluster with the smallest y value
    min_y = 0
    index_min_y = 0
    for i in range(max_label+1):
        if centroids[i][1] < min_y:
            min_y = centroids[i][1]
            index_min_y = i
    print(index_min_y)

    #find the cluster with the smallest z value
    min_z = 0
    index_min_z = 0
    for i in range(max_label+1):
        if centroids[i][2] < min_z:
            min_z = centroids[i][2]
            index_min_z = i
    print(index_min_z)     
     
     #convert to open3d pointcloud  
    cluster1 = o3d.geometry.PointCloud()
    cluster1.points = o3d.utility.Vector3dVector(clusters[1])
    cluster1.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw([cluster1])   
    o3d.io.write_point_cloud("cluster_libish6.ply", cluster1)     

    hull, _ = cluster1.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    o3d.visualization.draw([cluster1, hull_ls])

    axis_aligned_bounding_box = hull_ls.get_axis_aligned_bounding_box()
    axis_aligned_bounding_box.color = (1, 0, 0)
    obb = oriented_bounding_box = cluster1.get_oriented_bounding_box()
    oriented_bounding_box.color = (0, 1, 0)
    print(
        "Displaying axis_aligned_bounding_box in red and oriented bounding box in green ..."
    )
    o3d.visualization.draw(
        [outlier_cloud, axis_aligned_bounding_box, oriented_bounding_box]) 
    centre = oriented_bounding_box.get_center()
    #draw a sphere at the centre of the oriented bounding box
    # sphere =  o3d.geometry.create_mesh_sphere(radius=1.0, resolution=20)
    # sphere.compute_vertex_normals()
    # sphere.paint_uniform_color([0, 0, 1])
    # sphere.translate(centre, relative=False, copy=False)
    # o3d.visualization.read_selection_polygon_volume(oriented_bounding_box)
    # # o3d.visualization.draw([sphere])

    obb_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=100, origin=[centre[0]])
    mesh = obb_frame.translate(centre)
    obb_frame.rotate(obb.R, center=centre)
    o3d.visualization.draw_geometries([mesh, obb_frame])

    # mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
    # T = np.eye(4)
    # T[:3, :3] = mesh.get_rotation_matrix_from_xyz((0, np.pi / 3, np.pi / 2))
    # T[0, 3] = 1
    # T[1, 3] = 1.3
    # print(T)
    # mesh_t = copy.deepcopy(mesh).transform(T)
    # o3d.visualization.draw_geometries([mesh, mesh_t])

    #extract the centroid of the cluster
    #extract the pose of the cluster (centroid and orientation) 
    # get_rotation_matrix_from_quaternion(rotation: numpy.ndarray[numpy.float64[4, 1]]) → numpy.ndarray[numpy.float64[3, 3]]
    
    #convert centroid to open3d pointcloud
    centroid1 = o3d.geometry.PointCloud()
    centroid1.points = o3d.utility.Vector3dVector(centroids[1])
    centroid1.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw([centroid1])
    o3d.io.write_point_cloud("centroid_libish6.ply", centroid1) 

    # # getpose from boundingbox
    pickup_pose = np.identity(4)
    pickup_pose [:3, 3] = np.asarray(centroid1[1])
    pickup_pose [:3, :3] = np.asarray(oriented_bounding_box.get_rotation_matrix_from_quaternion()) 
    print(pickup_pose)

    #construct quaternion from rotation matrix
    quaternion = np.identity(4)
    quaternion[:3, 3] = np.asarray(centroid1[1])
    quaternion[:3, :3] = np.asarray(oriented_bounding_box.get_rotation_matrix_from_quaternion()) 
    print(quaternion)

    
    # # getpose from boundingbox
    # pose = np.identity(4)
    # pose[:3, 3] = np.asarray(centroid1[1])
    # pose[:3, :3] = np.asarray(oriented_bounding_box.get_rotation_matrix_from_quaternion())
    # print(pose)

    # #construct quaternion from rotation matrix
    # quaternion = np.identity(4)
    # quaternion[:3, 3] = np.asarray(centroid1[1])
    # quaternion[:3, :3] = np.asarray(oriented_bounding_box.get_rotation_matrix_from_quaternion())
    # print(quaternion)

    #the pose
    #  - The first three elements are the translation vector.   
    # - The next three elements are the rotation vector.
    # - The last three elements are the scale vector.
    # - The rotation vector is a unit quaternion in the (x, y, z, w) format.
           
    # #Find convex hull
    # hull = ConvexHull(outlier_cloud, qhull_options="QJ")
    # # o3d.visualization.draw_geometries([outlier_cloud]) 

    # # #Get centroid
    # cx = np.mean(hull.points[hull.vertices,0])
    # cy = np.mean(hull.points[hull.vertices,1])
    # cz = np.mean(hull.points[hull.vertices,2])

    # #Plot convex hull
    # for simplex in hull.simplices:
    #  points = labels
    # plt.plot(points[simplex, 0], points[simplex, 1], 'k-')

    #Plot centroid
    # plt.plot(cx, cy, cz,'x',ms=20)
    # plt.show()
