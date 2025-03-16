import open3d as o3d
import numpy as np
import colorsys
import matplotlib.pyplot as plt
import pyransac3d as pyrsc
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import json5
import csv

# EDIT THESE TO TOGGLE PLOTS
plot = False # Less important info
plot2 = True # Most important info

# EDIT PATH
results_path = 'C:\\Users\\eagle\\Documents\\Tech stuff\\FarmBot_Project_Aug22\\FarmBot_Project\\Intern Data\\Ray\\Data processing\\Pose estimation\\results.csv'
with open(results_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['','No. Ground truth','No. Found','No. Extra','Recall (%)','Superellipsoid','Paraboloid','Plane'])

for data_num in range(4,12):
    print("Plant " + str(data_num))

    # EDIT PATHS
    # Path to point cloud
    point_cloud_path = 'C:\\Users\\eagle\\Documents\\Tech stuff\\FarmBot_Project_Aug22\\FarmBot_Project\\FinalDatasets\\Data' + str(data_num) + '\\D' + str(data_num) + '.ply'
    # Path to AWS labeled bounding box
    bbox_path = 'C:\\Users\\eagle\\Documents\\Tech stuff\\FarmBot_Project_Aug22\\FarmBot_Project\\FinalDatasets\\Data' + str(data_num) + '\\output' + str(data_num) + '.json'
    scale = 25 # scale factor used in AWS labeling
    # Path to estimated bounding box using graph paper method
    data = np.load('C:\\Users\\eagle\\Documents\\Tech stuff\\FarmBot_Project_Aug22\\FarmBot_Project\\Final Results\\Test3 Final\\Data' + str(data_num) + '_arrays.npz', allow_pickle=True)

    def parse_json_file(file_path):
        try:
            with open(file_path, 'r') as file:
                # Load JSON data from file into a dictionary
                data_dict = json.load(file)
                return data_dict
        except FileNotFoundError:
            print("File not found:", file_path)
            return None
        except json.JSONDecodeError as e:
            print("Error decoding JSON:", e)
            return None

    def get_arrow(direction, start=np.array([0,0,0]), scale=1, color=np.array([0,0,0])):
        r, _ = R.align_vectors(direction[np.newaxis,:], np.array([[0,0,1]]))
        arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=scale*0.1, cone_radius=scale*0.2, cylinder_height=scale*2.0, cone_height=scale*1.0)
        arrow.rotate(r.as_matrix(), np.array([0,0,0]))
        arrow.translate(start)
        arrow.paint_uniform_color(color)
        return arrow

    pcd = o3d.io.read_point_cloud(point_cloud_path)
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

    draw_list = [pcd, axes]
    ground_truth_normals_list = []
    ground_truth_bbox_list = []

    store = parse_json_file(bbox_path)
    answers = store["answers"]
    for answer in answers:
        boundingCuboids = answer["answerContent"]["boundingCuboids"]["boundingCuboids"]

        for bb in boundingCuboids:

            # Get values
            objectName = bb["objectName"]

            centerX = bb["centerX"] / scale # scale back values that were scaled up before labeling in AWS
            centerY = bb["centerY"] / scale
            centerZ = bb["centerZ"] / scale

            height = bb["height"] / scale
            length = bb["length"] / scale
            width = bb["width"] / scale

            roll = bb["roll"]
            pitch = bb["pitch"]
            yaw = bb["yaw"]
            
            center = np.array([centerX, centerY, centerZ])
            direction = np.array([roll, pitch, yaw])
            extent = np.array([length, width, height])
            # extent = np.array([width, height, length]) # ???

            # Calculate normal vector and bounding box
            r = R.from_euler('XYZ',direction)
            normal = r.apply(np.array([1,0,0]))
            arrow = get_arrow(normal,start=center,scale=0.05)
            # scale = 0.1
            # arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=scale*0.1, cone_radius=scale*0.2, cylinder_height=scale*2.0, cone_height=scale*1.0)
            # rot = R.from_euler('y', 90, degrees=True)
            # arrow.rotate(rot.as_matrix(),center=np.array([0,0,0]))
            # rot = R.from_euler('XYZ',direction)
            # arrow.rotate(rot.as_matrix(),center=np.array([0,0,0]))
            # arrow.translate(center)

            bbox = o3d.geometry.OrientedBoundingBox(center=center, R=r.as_matrix(), extent=extent)

            draw_list.append(arrow)
            ground_truth_bbox_list.append(bbox)

            pcd_crop = pcd.crop(bbox)
            ground_truth_normals_list.append(normal)

    draw_list = draw_list + ground_truth_bbox_list
    if plot:
        o3d.visualization.draw_geometries(draw_list, zoom=0.6999, front=[0, 0, 1], lookat=np.mean(np.asarray(pcd.points), axis=0), up=[0, 1, 0], left=0, top=0)
    

    # UNUSED SINCE ONLY PETALS ARE USED
    soil_center = np.array([0,0,0])
    # Center point cloud at the center of the soil (bottom of plant)
    # plane = pyrsc.Plane()
    # eq,inliers = plane.fit(np.asarray(pcd.points), thresh=0.1,maxIteration=80)
    # soil_points = np.asarray(pcd.points)[inliers]
    # soil_colors = np.asarray(pcd.colors)[inliers]
    # soil_center = np.mean(soil_points,axis=0)
    # if plot:
    #     axes2 = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=soil_center)
    #     pcd_soil = o3d.geometry.PointCloud()
    #     pcd_soil.points = o3d.utility.Vector3dVector(soil_points)
    #     pcd_soil.colors = o3d.utility.Vector3dVector(soil_colors)
    #     o3d.visualization.draw_geometries([axes2,pcd_soil], zoom=2.0, front=[0, 0, 1], lookat=np.mean(np.asarray(pcd.points), axis=0), up=[0, 1, 0], left=0, top=0)

    estimated_bbox_list = []
    for key in data.files:
        flower_info = data[key].item()
        bbox = o3d.geometry.OrientedBoundingBox(center=flower_info['center'], R=flower_info['R'], extent=flower_info['extent'])
        estimated_bbox_list.append(bbox)

    # Visualize estimated bounding boxes in yellow
    if plot2:
        for estimated_bbox in estimated_bbox_list:
            # Draw yellow bounding boxes
            bbox_lines = o3d.geometry.LineSet.create_from_oriented_bounding_box(estimated_bbox)
            bbox_lines.colors = o3d.utility.Vector3dVector([[1,0,0]] * len(bbox_lines.lines))
            draw_list.append(bbox_lines)
        o3d.visualization.draw_geometries(draw_list, zoom=0.6999, front=[0, 0, 1], lookat=np.mean(np.asarray(pcd.points), axis=0), up=[0, 1, 0], left=0, top=0)
            
    # Match ground truth bbox with estimated bbox
    final_bbox_list = []
    for idx,ground_truth_bbox in enumerate(ground_truth_bbox_list):
        min_distance = 0.02 # threshold for matching estimated bbox with ground truth bbox
        found = False
        best_estimated_bbox = None
        for estimated_bbox in estimated_bbox_list:
            distance = np.linalg.norm(ground_truth_bbox.center - estimated_bbox.center)
            if distance < min_distance:
                min_distance = distance
                best_estimated_bbox = estimated_bbox
                found = True
        if found:
            final_bbox_list.append(best_estimated_bbox)
        else:
            ground_truth_normals_list[idx] = None # mark the normal for that index to be removed
    ground_truth_normals_list = [normal for normal in ground_truth_normals_list if normal is not None] # remove all unused normals


    # Count number of ground truth flowers found and number of extra flowers
    num_ground_truth = len(ground_truth_bbox_list)
    num_found = len(final_bbox_list)
    num_extra = len(estimated_bbox_list) - num_found

    # Plot estimated bboxes with ground truth normals and crop flowers
    draw_list_2 = [pcd, axes]
    flowers_list = []
    for idx,bbox in enumerate(final_bbox_list):
        # Draw yellow bounding boxes
        bbox_lines = o3d.geometry.LineSet.create_from_oriented_bounding_box(bbox)
        bbox_lines.colors = o3d.utility.Vector3dVector([[1,1,0]] * len(bbox_lines.lines))
        draw_list_2.append(bbox_lines)

        # Draw arrows too
        arrow = get_arrow(ground_truth_normals_list[idx],start=bbox.center,scale=0.05)
        draw_list_2.append(arrow)

        # Crop flowers
        pcd_crop = pcd.crop(bbox)
        flowers_list.append(pcd_crop)

    if plot:
        o3d.visualization.draw_geometries(draw_list_2, zoom=0.6999, front=[0, 0, 1], lookat=np.mean(np.asarray(pcd.points), axis=0), up=[0, 1, 0], left=0, top=0)

    color = [
        [255, 0, 0],   # Red
        [0, 255, 0],   # Green
        [0, 0, 255],   # Blue
        [255, 255, 0], # Yellow
        [255, 0, 255], # Magenta
        [0, 255, 255], # Cyan
        [0, 128, 0],   # Green (dark)
        [0, 0, 128],   # Navy
        [128, 128, 0],  # Olive
        [128, 0, 128],  # Purple
        [0, 128, 128],  # Teal
        [128, 128, 128],# Gray
        [255, 165, 0],  # Orange
        [255, 192, 203],# Pink
        [0, 0, 0],      # Black
        [128, 0, 0],    # Maroon (dark)
        [0, 128, 0],    # Green (medium)
        [0, 128, 128],
        [128, 128, 128]
    ]
    # HSV filtering on bounding boxes to get petal and pistil
    def convert_rgb_2_hsv(all_rgb_colors):
        all_hsv_colors = []

        for i in range(len(all_rgb_colors)):
            temp_color = all_rgb_colors[i]
            temp = colorsys.rgb_to_hsv(temp_color[0], temp_color[1], temp_color[2])
            all_hsv_colors.append(temp)

        all_hsv_colors = np.asarray(all_hsv_colors)

        return all_hsv_colors

    def hsv_filter(hsv_color): #flower
        
        # Our red strawberry flowers have a very particular color of petal
        # We will segment the flowers out based on this, firstly
        # --
        # H,S,V = {(300, 0] U [0, 60), (0.3, 1), (0.3, 1)}

        low_h1 = 0/360
        high_h1 = 40/360
        low_h2 = 80/360
        high_h2 = 360/360

        low_s = 0
        high_s = 0.8

        low_v = 0
        high_v = 1


        if (low_h1 <= hsv_color[0] <= high_h1 or low_h2 <= hsv_color[0] <= high_h2) and low_s <= hsv_color[1] <= high_s and low_v <= hsv_color[2] <= high_v:
            return True
        else:
            return False

    def hsv_filter_petal(hsv_color):
        # Filters out pistil and stem
        low_reject_h = 40/360
        high_reject_h = 140/360

        low_reject_s = 0.15
        high_reject_s = 1.0

        low_reject_v = 0.0
        high_reject_v = 0.75

        if (low_reject_v <= hsv_color[2] <= high_reject_v):
            return False
        elif (low_reject_h <= hsv_color[0] <= high_reject_h) and (low_reject_s <= hsv_color[1] <= high_reject_s):
            return False
        else:
            return True
        
    def hsv_filter_pistil(hsv_color):
        # Only get pistil
        low_h = 40/360
        high_h = 63/360

        low_s = 0.30
        high_s = 1.0

        low_v = 0.65
        high_v = 1.0

        if (low_h <= hsv_color[0] <= high_h) and (low_s <= hsv_color[1] <= high_s) and (low_v <= hsv_color[2] <= high_v):
            return True
        else:
            return False

    petals_list = []
    pistils_list = []
    for idx,flower in enumerate(flowers_list):
        # print("Flower:",idx)
        points = np.asarray(flower.points)
        colors = np.asarray(flower.colors)
        hsv_colors = convert_rgb_2_hsv(colors)

        # Petal data
        filtered_points = points[np.array([hsv_filter_petal(hsv_color) for hsv_color in hsv_colors], dtype=bool),:]
        filtered_colors = colors[np.array([hsv_filter_petal(hsv_color) for hsv_color in hsv_colors], dtype=bool),:]
        # filtered_points = points[np.array([not(hsv_filter_petal(hsv_color)) for hsv_color in hsv_colors], dtype=bool),:]
        # filtered_colors = colors[np.array([not(hsv_filter_petal(hsv_color)) for hsv_color in hsv_colors], dtype=bool),:]
        # filtered_points = []
        # filtered_colors = []
        # for idx in range(hsv_colors.shape[0]):
        #     point = points[idx]
        #     color = colors[idx]
        #     hsv_color = hsv_colors[idx]
        #     if hsv_filter(hsv_color):
        #         filtered_points.append(point)
        #         filtered_colors.append(color)
        pcd_petal = o3d.geometry.PointCloud()
        pcd_petal.points = o3d.utility.Vector3dVector(filtered_points)
        pcd_petal.colors = o3d.utility.Vector3dVector(filtered_colors)
        petals_list.append(pcd_petal)

        # Pistil data
        pistil_potential_points = points[np.array([hsv_filter_pistil(hsv_color) for hsv_color in hsv_colors], dtype=bool),:]
        pistil_potential_colors = colors[np.array([hsv_filter_pistil(hsv_color) for hsv_color in hsv_colors], dtype=bool),:]
        # pistil_potential_points = points[np.array([not(hsv_filter_pistil(hsv_color)) for hsv_color in hsv_colors], dtype=bool),:]
        # pistil_potential_colors = colors[np.array([not(hsv_filter_pistil(hsv_color)) for hsv_color in hsv_colors], dtype=bool),:]
        pcd_potential_pistil = o3d.geometry.PointCloud()
        pcd_potential_pistil.points = o3d.utility.Vector3dVector(pistil_potential_points)
        pcd_potential_pistil.colors = o3d.utility.Vector3dVector(pistil_potential_colors)

        # DBscan to get cluster closest to center of petals
        labels = np.array(pcd_potential_pistil.cluster_dbscan(eps=0.005, min_points=5))
        best_cluster = None
        smallest_distance = np.inf
        draw_list = []
        for i in range(labels.max()+1):
            cluster_points = np.asarray(pcd_potential_pistil.points)[labels==i]
            cluster_colors = np.asarray(pcd_potential_pistil.colors)[labels==i]
            viz_colors = np.full_like(cluster_colors, color[i]) # temp
            pcd_cluster = o3d.geometry.PointCloud()
            pcd_cluster.points = o3d.utility.Vector3dVector(cluster_points)
            pcd_cluster.colors = o3d.utility.Vector3dVector(viz_colors)
            draw_list.append(pcd_cluster)

            distance = np.linalg.norm(np.mean(cluster_points,axis=0) - np.mean(filtered_points,axis=0))
            # avg_colors = np.mean(cluster_colors,axis=0)
            # h, s, v = colorsys.rgb_to_hsv(avg_colors[0], avg_colors[1], avg_colors[2])
            # distance = np.abs(h - 55)
            # print("Cluster:",i,"Distance:",distance)
            if distance < smallest_distance:
                best_cluster = i
                smallest_distance = distance
        # print("Best cluster:",best_cluster)
        pcd_pistil = o3d.geometry.PointCloud()
        pcd_pistil.points = o3d.utility.Vector3dVector(np.asarray(pcd_potential_pistil.points)[labels==best_cluster])
        pcd_pistil.colors = o3d.utility.Vector3dVector(np.asarray(pcd_potential_pistil.colors)[labels==best_cluster])

        pistils_list.append(pcd_pistil)

        if plot:
            print("Flower:",idx)
            # Visualization
            o3d.visualization.draw_geometries([flower], zoom=2.0, front=[0, 0, 1], lookat=np.mean(np.asarray(flower.points), axis=0), up=[0, 1, 0], left=0, top=0)
            o3d.visualization.draw_geometries([pcd_petal], zoom=2.0, front=[0, 0, 1], lookat=np.mean(np.asarray(flower.points), axis=0), up=[0, 1, 0], left=0, top=0)
            o3d.visualization.draw_geometries([pcd_potential_pistil], zoom=2.0, front=[0, 0, 1], lookat=np.mean(np.asarray(flower.points), axis=0), up=[0, 1, 0], left=0, top=0)
            o3d.visualization.draw_geometries(draw_list, zoom=2.0, front=[0, 0, 1], lookat=np.mean(np.asarray(flower.points), axis=0), up=[0, 1, 0], left=0, top=0)
            o3d.visualization.draw_geometries([pcd_petal, pcd_pistil], zoom=2.0, front=[0, 0, 1], lookat=np.mean(np.asarray(flower.points), axis=0), up=[0, 1, 0], left=0, top=0)
            # option = flower
            # vis = o3d.visualization.VisualizerWithEditing()
            # vis.create_window()
            # vis.add_geometry(option)
            # vis.run()
            # vis.destroy_window()
            # for point in vis.get_picked_points():
            #     temp_color = option.colors[point]
            #     temp = colorsys.rgb_to_hsv(temp_color[0], temp_color[1], temp_color[2])
            #     h, s, v = temp
            #     h = h*360
            #     print(h, s, v)

            # option = pcd_petal
            # vis = o3d.visualization.VisualizerWithEditing()
            # vis.create_window()
            # vis.add_geometry(option)
            # vis.run()
            # vis.destroy_window()
            # for point in vis.get_picked_points():
            #     temp_color = option.colors[point]
            #     temp = colorsys.rgb_to_hsv(temp_color[0], temp_color[1], temp_color[2])
            #     h, s, v = temp
            #     h = h*360
            #     print(h, s, v)

    # Fitting shapes onto flower

    class FitShape:
        def __init__(self, points, colors, pistil_direction, combined_direction):
            self.points = points
            self.colors = colors
            self.pistil_direction = pistil_direction
            self.combined_direction = combined_direction
            self.normal = np.array([0,0,0])
            self.front = np.array([0,0,0])
            self.euler_angles = np.array([0,0,0])
            self.surface = (0,0,0)
            self.success = True
        
        def c_func(self,w,m):
            return np.sign(np.cos(w)) * np.abs(np.cos(w))**m

        def s_func(self,w,m):
            return np.sign(np.sin(w)) * np.abs(np.sin(w))**m

        def rotate_xyz(self,x,y,z,roll,pitch,yaw,reversed=False):
            datashape = x.shape
            if reversed:
                r = R.from_euler('zyx',np.array([-yaw, -pitch, -roll]))
            else:
                r = R.from_euler('xyz',np.array([roll, pitch, yaw]))
            xyz = r.apply(np.array([x.flatten(),y.flatten(),z.flatten()]).T)
            x,y,z = xyz.T
            return x.reshape(datashape), y.reshape(datashape), z.reshape(datashape)

        def translate_xyz(self,x,y,z,Tx,Ty,Tz):
            x += Tx
            y += Ty
            z += Tz
            return x,y,z
        
        def get_euler_angles(self, original, transformed):
            # Compute the rotation matrix using least squares optimization
            r, _ = R.align_vectors(original, transformed)
            euler_angles = r.as_euler('xyz', degrees=True)
            return euler_angles

        # superellipsoid -> x,y,z : https://en.wikipedia.org/wiki/Superellipsoid
        def superellipsoid_to_xyz(self,A=1,B=1,C=1,E1=0.5,E2=0.3,custom_args=None):
            if custom_args is None:
                v = np.linspace(-np.pi/2, np.pi/2, 30)
                u = np.linspace(-np.pi, np.pi, 50)
            else:
                v = np.linspace(custom_args["v_min"], custom_args["v_max"], custom_args["v_count"])
                u = np.linspace(custom_args["u_min"], custom_args["u_max"], custom_args["u_count"])
            uu, vv = np.meshgrid(u,v)
            
            #r = 2    # from wikipedia
            #t = 2.5  # from wikipedia
            r = 2/E2  # from the paper
            t = 2/E1  # from the paper
            
            x = A * self.c_func(vv, 2/t) * self.c_func(uu, 2/r)
            y = B * self.c_func(vv, 2/t) * self.s_func(uu, 2/r)
            z = C * self.s_func(vv, 2/t)
            
            return x,y,z
        
        def ellipsoid_to_xyz(self,A=1,B=1,C=1,custom_args=None):
            u = np.linspace(0, 2*np.pi, 100)
            v = np.linspace(0, np.pi, 100)
            uu, vv = np.meshgrid(u,v)
            
            x = A * np.cos(uu) * np.sin(vv)
            y = B * np.sin(uu) * np.sin(vv)
            z = C * np.cos(vv)
            
            return x,y,z
        
        def paraboloid_to_xyz(self,A=1,B=1,custom_args=None):
            u = np.linspace(-0.05, 0.05, 100)
            v = np.linspace(-0.05, 0.05, 100)
            x, y = np.meshgrid(u,v)
            
            z = (x/A)**2 + (y/B)**2
            
            return x,y,z
        
        def plane_to_xyz(self,A=1,B=1,C=1,custom_args=None):
            u = np.linspace(-0.05, 0.05, 100)
            v = np.linspace(-0.05, 0.05, 100)
            x, y = np.meshgrid(u,v)
            
            z = -(A*x + B*y)/C
            
            return x,y,z

        def loss_superellipsoid(self,x0, x, y, z):
            a,b,c,e1,e2,roll,pitch,yaw = x0
            # print("Params: {}".format(x0))
            
            x,y,z = self.rotate_xyz(x,y,z,roll,pitch,yaw)
            #x,y,z = x+0j, y+0j, z+0j
            x,y,z = np.abs(x), np.abs(y), np.abs(z)

            f = ( (x/a)**(2/e2) + (y/b)**(2/e2) )**(e2/e1) + (z/c)**(2/e1)
            loss = (np.abs(a*b*c)) * (f**e1 - 1.)**2
            
            # additions: (MAYBE? OR ADD THESE AS A CONSTRAINT)
            #f += 0.001*np.abs(tx + ty + tz)
            #f += 0.001*np.abs(roll+pitch+yaw)
            #f += 0.001*(a-1)
            #f += 0.001*(b-1)
            #f += 0.001*(c-1)

            #f = np.sum(f)
            #f = np.linalg.norm(f)
            #print(f)
            return loss

        def loss_ellipsoid(self,x0, x, y, z):
            a,b,c,roll,pitch,yaw = x0
            # print("Params: {}".format(x0))
            
            x,y,z = self.rotate_xyz(x,y,z,roll,pitch,yaw)
            #x,y,z = x+0j, y+0j, z+0j
            x,y,z = np.abs(x), np.abs(y), np.abs(z)

            f = (x/a)**2 + (y/b)**2 + (z/c)**2
            loss = (f - 1.)**2
            
            # additions: (MAYBE? OR ADD THESE AS A CONSTRAINT)
            #f += 0.001*np.abs(tx + ty + tz)
            #f += 0.001*np.abs(roll+pitch+yaw)
            #f += 0.001*(a-1)
            #f += 0.001*(b-1)
            #f += 0.001*(c-1)

            #f = np.sum(f)
            #f = np.linalg.norm(f)
            #print(f)
            return loss

        def loss_paraboloid(self,x0, x, y, z):
            a,b,roll,pitch,yaw = x0
            # print("Params: {}".format(x0))
            
            x,y,z = self.rotate_xyz(x,y,z,roll,pitch,yaw)
            # x, y, z = self.translate_xyz(x, y, z,tx,ty,tz)
            #x,y,z = x+0j, y+0j, z+0j
            x,y,z = np.abs(x), np.abs(y), np.abs(z)
            f = (x/a)**2 + (y/b)**2
            loss = (z - f)**2
            return loss
        
        def find_normal_direction(self,normal):
            # Only using pistil for final version
            dot_product = np.dot(normal/np.linalg.norm(normal),self.pistil_direction/np.linalg.norm(self.pistil_direction))
            # print("Dot product {}".format(dot_product))
            # if np.abs(dot_product) < 0.4: # Use combination of soil and pistil if pistil alone doesn't give a good indication of direction
            #     dot_product_2 = np.dot(normal/np.linalg.norm(normal),self.combined_direction/np.linalg.norm(self.combined_direction))
            #     # print("Dot product 2 {}".format(dot_product_2))
            #     if dot_product_2 < 0: # Flip normal
            #         normal = -normal
            # elif dot_product < 0: # Flip normal
            #     normal = -normal
            if dot_product < 0: # Flip normal
                normal = -normal
            return normal


        def fit(self,shape='superellipsoid',x0=None,plot=False):
            if shape=='superellipsoid':
                if x0==None:
                    x0 = (0.005,0.005,0.005,   1.0,1.0,    0,0,0)

                # Fit using least squares
                bounds = ([0, 0, 0, 0.9, 0.9, -np.inf, -np.inf, -np.inf],
                            [0.1, 0.1, 0.1, 1.1, 1.1, np.inf, np.inf, np.inf])
                result = least_squares(self.loss_superellipsoid, x0, bounds=bounds, args=(self.points[:,0], self.points[:,1], self.points[:,2]), method="trf")
                if np.isnan(result["x"]).any():
                    print("Superellipsoid fitting failed")
                    self.success = False
                    return
                A,B,C,E1,E2,roll,pitch,yaw = result["x"] # Note that rotation angles describe rotation from points to standard coordinate frame
                # print("A:{},B:{},C:{},E1:{},E2:{},yaw:{},pitch:{},roll:{}".format(A,B,C,E1,E2,yaw,pitch,roll))

                # Generate superellipsoid surface for plotting
                xr, yr, zr = self.superellipsoid_to_xyz(A,B,C,E1,E2,custom_args=None)
                xr, yr, zr = self.rotate_xyz(xr, yr, zr,roll,pitch,yaw,reversed=True)
                # xr, yr, zr = self.translate_xyz(xr, yr, zr,-tx,-ty,-tz)
                self.surface = (xr,yr,zr)

                # Get "most squished" axis and apply rotation to it
                r = R.from_euler('xyz',np.array([roll, pitch, yaw]))
                index = np.argmin(np.array([A,B,C]))
                axis = np.eye(3)[:,index]
                normal = r.apply(axis,inverse=True)

                # Choose correct normal out of two possible options
                normal = self.find_normal_direction(normal)

                self.normal = normal
                # self.front = front
                # print('Calculated normal: {}'.format(np.round(normal, 5)))

                # Get euler angles
                # front_top = np.array([[1,0,0],
                #                [0,0,1]])
                # front_top_transformed = np.vstack((front, normal))
                # self.euler_angles = self.get_euler_angles(front_top, front_top_transformed)
            elif shape=='ellipsoid':
                if x0==None:
                    x0 = (0.005,0.005,0.005,    0,0,0)

                # Fit using least squares
                bounds = ([0, 0, 0, -np.inf, -np.inf, -np.inf],
                            [0.1, 0.1, 0.1, np.inf, np.inf, np.inf])
                result = least_squares(self.loss_ellipsoid, x0, bounds=bounds, args=(self.points[:,0], self.points[:,1], self.points[:,2]), method="trf")
                if np.isnan(result["x"]).any():
                    print("Ellipsoid fitting failed")
                    self.success = False
                    return
                A,B,C,roll,pitch,yaw = result["x"] # Note that rotation angles describe rotation from points to standard coordinate frame
                # print("A:{},B:{},C:{},E1:{},E2:{},yaw:{},pitch:{},roll:{}".format(A,B,C,E1,E2,yaw,pitch,roll))

                # Generate superellipsoid surface for plotting
                xr, yr, zr = self.ellipsoid_to_xyz(A,B,C,custom_args=None)
                xr, yr, zr = self.rotate_xyz(xr, yr, zr,roll,pitch,yaw,reversed=True)
                # xr, yr, zr = self.translate_xyz(xr, yr, zr,-tx,-ty,-tz)
                self.surface = (xr,yr,zr)

                # Get "most squished" axis and apply rotation to it
                r = R.from_euler('xyz',np.array([roll, pitch, yaw]))
                index = np.argmin(np.array([A,B,C]))
                axis = np.eye(3)[:,index]
                normal = r.apply(axis,inverse=True)

                # Choose correct normal out of two possible options
                normal = self.find_normal_direction(normal)

                self.normal = normal
                # self.front = front
                # print('Calculated normal: {}'.format(np.round(normal, 5)))

                # Get euler angles
                # front_top = np.array([[1,0,0],
                #                [0,0,1]])
                # front_top_transformed = np.vstack((front, normal))
                # self.euler_angles = self.get_euler_angles(front_top, front_top_transformed)
            elif shape=='paraboloid':
                if x0==None:
                    x0 = (0.01,0.01,    0,0,0)

                # Fit using least squares
                result = least_squares(self.loss_paraboloid, x0, args=(self.points[:,0], self.points[:,1], self.points[:,2]), method="lm")
                if np.isnan(result["x"]).any():
                    print("Paraboloid fitting failed")
                    self.success = False
                    return
                A,B,roll,pitch,yaw = result["x"] # Note that rotation angles describe rotation from points to standard coordinate frame
                # print("A:{},B:{},yaw:{},pitch:{},roll:{}".format(A,B,yaw,pitch,roll))

                # Generate paraboloid surface for plotting
                xr, yr, zr = self.paraboloid_to_xyz(A,B,custom_args=None)
                xr, yr, zr = self.rotate_xyz(xr, yr, zr,roll,pitch,yaw,reversed=True)
                # xr, yr, zr = self.translate_xyz(xr, yr, zr,-tx,-ty,-tz)
                self.surface = (xr,yr,zr)

                # Since a paraboloid is directional, we can rotate (0,0,1) to get the normal vector
                r = R.from_euler('xyz',np.array([roll, pitch, yaw]))
                normal = r.apply(np.array([0,0,1]),inverse=True)
                # if normal[2] < 0: normal *= -1          # Ensure normal vector always points upward
                self.normal = normal
                self.front = None
                # print('Calculated normal: {}'.format(np.round(normal, 5)))

                # In this case the euler angles are simply roll, pitch, yaw
                self.euler_angles = (roll, pitch, yaw)
            elif shape=='plane':
                Cov = np.cov(self.points, rowvar=False)          # Covariance matrix of centroid-removed points
                [val, vec] = np.linalg.eig(Cov)         # Eigen values and vectors of points
                normal = vec[:, np.argmin(val)]
                # print("A:{},B:{},C:{}".format(normal[0],normal[1],normal[2]))

                # Choose correct normal out of two possible options
                normal = self.find_normal_direction(normal)

                # Generate plane surface for plotting
                xr, yr, zr = self.plane_to_xyz(normal[0],normal[1],normal[2],custom_args=None)
                self.surface = (xr,yr,zr)

                self.normal = normal
                self.front = None
                # print('Calculated normal: {}'.format(np.round(normal, 5)))

                # Get euler angles
                top = np.array([0,0,1])
                self.euler_angles = self.get_euler_angles(top[np.newaxis,:], normal[np.newaxis,:])
            elif shape=='ransac_plane':
                plane = pyrsc.Plane()
                eq,_ = plane.fit(points, 0.01)
                normal = np.asarray(eq[:3])

                # Choose correct normal out of two possible options
                normal = self.find_normal_direction(normal)

                # Generate plane surface for plotting
                xr, yr, zr = self.plane_to_xyz(normal[0],normal[1],normal[2],custom_args=None)
                self.surface = (xr,yr,zr)

                self.normal = normal
                self.front = None
                # print('Calculated normal: {}'.format(np.round(normal, 5)))

                # Get euler angles
                top = np.array([0,0,1])
                self.euler_angles = self.get_euler_angles(top[np.newaxis,:], normal[np.newaxis,:])

            if plot:

                # PLOT
                fig = plt.figure(figsize=(8,6))
                ax = fig.add_subplot(projection='3d') 
                lim = 0.05
                ax.set_xlim(-lim, lim)
                ax.set_ylim(-lim, lim)
                ax.set_zlim(-lim, lim)
                # ax.set_box_aspect([1,1,1]) 
                ax.plot_surface(xr, yr, zr, alpha=0.5, color='red')
                _=ax.scatter(self.points[:,0], self.points[:,1], self.points[:,2], c=self.colors, alpha=1)
                ax.quiver(0, 0, 0, normal[0], normal[1], normal[2], length=np.max(np.linalg.norm(self.points, axis=1)), colors=np.array([1,0,0,1]))
                plt.show(block=True)

    def get_surface(surface,center=np.array([0,0,0]), color=np.array([0,0,0])):
        x = surface[0].flatten() + center[0]
        y = surface[1].flatten() + center[1]
        z = surface[2].flatten() + center[2]

        vertices = np.vstack([x, y, z]).T
        vertices = np.array(vertices)
        
        # Vertices
        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(vertices)
        
        # Triangles
        faces = []
        resolution = 100
        for i in range(resolution-1):
            for j in range(resolution-1):
                p1 = i * resolution + j
                p2 = i * resolution + (j + 1)
                p3 = (i + 1) * resolution + (j + 1)
                p4 = (i + 1) * resolution + j
                faces.append([p1, p2, p3])
                faces.append([p1, p3, p4])
        mesh.triangles = o3d.utility.Vector3iVector(faces)

        # Color
        mesh.paint_uniform_color(color)

        return mesh

    draw_list = [pcd,axes]
    angle_errors = np.full((len(flowers_list),3),np.nan)
    for idx in range(len(flowers_list)):
        print("Flower {}".format(idx))
        # points = np.asarray(flower.points) * 5/6 # scaling factor to match real world dimensions
        # Petal data
        points = np.asarray(petals_list[idx].points)
        colors = np.asarray(petals_list[idx].colors)
        center = np.mean(points, axis=0)
        # Get pistil and soil direction RELATIVE TO FLOWER NOT PETALS
        pistil_points = np.asarray(pistils_list[idx].points)
        pistil_direction = np.mean(pistil_points, axis=0) - np.mean(np.asarray(flowers_list[idx].points),axis=0)
        pistil_direction = pistil_direction / np.linalg.norm(pistil_direction)
        soil_direction = np.mean(np.asarray(flowers_list[idx].points),axis=0) - soil_center
        soil_direction = soil_direction / np.linalg.norm(soil_direction)
        combined_direction = pistil_direction + soil_direction
        combined_direction = combined_direction / np.linalg.norm(combined_direction)


        points = points - center # center points at origin
        
        # Get ground truth
        ground_truth_normal = ground_truth_normals_list[idx]
        
        # Fit shapes: superellisoid, paraboloid, plane
            
        superellipsoid = FitShape(points, colors, pistil_direction, combined_direction)
        superellipsoid.fit(shape='superellipsoid')
        paraboloid = FitShape(points, colors, pistil_direction, combined_direction)
        paraboloid.fit(shape='paraboloid')
        plane = FitShape(points, colors, pistil_direction, combined_direction)
        plane.fit(shape='plane')

        # Get angle between vectors
        if superellipsoid.success: # superellipsoid can sometimes fail
            angle_superellipsoid = np.degrees(np.arccos(np.dot(ground_truth_normal, superellipsoid.normal)))
        else:
            superellipsoid.normal = np.array(np.nan)
            angle_superellipsoid = np.array(np.nan)
        angle_paraboloid = np.degrees(np.arccos(np.dot(ground_truth_normal, paraboloid.normal)))
        angle_plane = np.degrees(np.arccos(np.dot(ground_truth_normal, plane.normal)))
        print('Ground truth normal: {}, superellipsoid: {}, paraboloid: {}, plane: {}'.format(np.round(ground_truth_normal, 5), np.round(superellipsoid.normal, 5), np.round(paraboloid.normal, 5), np.round(plane.normal, 5)))
        print("Angle error: superellipsoid: {}, paraboloid: {}, plane: {}".format(np.round(angle_superellipsoid,3), np.round(angle_paraboloid,3), np.round(angle_plane,3)))
        angle_errors[idx,:] = np.array([angle_superellipsoid, angle_paraboloid, angle_plane])
        
        # Plot result
        c = np.array([[0,0,0,1],
            [1,0,0,1],
            [0,1,0,1],
            [0,0,1,1],
            [1,1,0,1],
            [1,0,1,1],
            [0,1,1,1]])
        if plot:
            fig = plt.figure(figsize=(8,6))
            ax = fig.add_subplot(projection='3d') 
            lim = 0.05
            ax.set_xlim(-lim, lim)
            ax.set_ylim(-lim, lim)
            ax.set_zlim(-lim, lim)
            ax.scatter(points[:,0], points[:,1], points[:,2], c=colors, alpha=1)
            ax.quiver(0, 0, 0, ground_truth_normal[0], ground_truth_normal[1], ground_truth_normal[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[0], arrow_length_ratio=0.1)
            if superellipsoid.success:
                ax.quiver(0, 0, 0, superellipsoid.normal[0], superellipsoid.normal[1], superellipsoid.normal[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[1], arrow_length_ratio=0.1)
                ax.plot_surface(superellipsoid.surface[0], superellipsoid.surface[1], superellipsoid.surface[2], alpha=0.05, color='red')
            ax.quiver(0, 0, 0, paraboloid.normal[0], paraboloid.normal[1], paraboloid.normal[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[2], arrow_length_ratio=0.1)
            ax.plot_surface(paraboloid.surface[0], paraboloid.surface[1], paraboloid.surface[2], alpha=0.05, color='green')
            ax.quiver(0, 0, 0, plane.normal[0], plane.normal[1], plane.normal[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[3], arrow_length_ratio=0.1)
            ax.plot_surface(plane.surface[0], plane.surface[1], plane.surface[2], alpha=0.05, color='blue')
            ax.quiver(0, 0, 0, soil_direction[0], soil_direction[1], soil_direction[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[4], arrow_length_ratio=0.1)
            ax.quiver(0, 0, 0, pistil_direction[0], pistil_direction[1], pistil_direction[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[5], arrow_length_ratio=0.1)
            ax.quiver(0, 0, 0, combined_direction[0], combined_direction[1], combined_direction[2], length=2*np.max(np.linalg.norm(points, axis=1)), colors=c[6], arrow_length_ratio=0.1)
            plt.show(block=True)

        # Draw arrows at the appropriate flower position
        draw_list.append(get_arrow(ground_truth_normal, start=center, scale=0.05, color=c[0,:3]))
        if superellipsoid.success:
            draw_list.append(get_arrow(superellipsoid.normal, start=center, scale=0.05, color=c[1,:3]))
        draw_list.append(get_arrow(paraboloid.normal, start=center, scale=0.05, color=c[2,:3]))
        draw_list.append(get_arrow(plane.normal, start=center, scale=0.05, color=c[3,:3]))
        draw_list.append(get_arrow(soil_direction, start=center, scale=0.05, color=c[4,:3]))
        draw_list.append(get_arrow(pistil_direction, start=center, scale=0.05, color=c[5,:3]))
        draw_list.append(get_arrow(combined_direction, start=center, scale=0.05, color=c[6,:3]))


        # Draw shapes at the appropriate flower position
        # draw_list.append(get_surface(superellipsoid.surface, center=center, color=c[1,:3]))
        # draw_list.append(get_surface(paraboloid.surface, center=center, color=c[2,:3]))
        # draw_list.append(get_surface(plane.surface, center=center, color=c[3,:3]))

    # Visualize final output
    if plot:
        o3d.visualization.draw_geometries(draw_list, zoom=0.6999, front=[0, 0, 1], lookat=np.mean(np.asarray(pcd.points), axis=0), up=[0, 1, 0], left=0, top=0)

    # Save results in file
    error_write = np.round(angle_errors,3).tolist()
    error_write = [['']*5 + sublist for sublist in error_write]
    with open(results_path, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Plant ' + str(data_num), num_ground_truth, num_found, num_extra, num_found/num_ground_truth*100])
        writer.writerows(error_write)