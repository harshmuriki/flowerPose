# %%
from roboflow import Roboflow
import open3d as o3d
import numpy as np
import cv2
from PIL import Image
import time
import colorsys
import os

# %%
show_pcl = True

# %%
def visualizer(cloud):

    if type(cloud) != list:
        cloud = [cloud]

    if show_pcl:
        # o3d.visualization.draw_geometries(cloud,
        #                             zoom=0.57899999999999985,
        #                             front=  [ 0.053781796277127751, 0.99511863815317547, 0.082743062484869914 ],
        #                             up =  [ -0.99452345262671604, 0.045944145215534776, 0.093873519673282182 ],
        #                             lookat=[ 0.14950467828195535, -0.21448131248991498, 0.63221199653621662 ])
        
        center_of_mass = np.mean(np.asarray(cloud[0].points), axis=0)
        
        # print("com", center_of_mass)
        o3d.visualization.draw_geometries(cloud,
                                    zoom=0.7,
                                    front=[0, 1, 0],
                                    lookat=center_of_mass,
                                    up=[0, 0, 1])
    

# %%
def object_detection(img_path, model):

    prediction_data = model.predict(img_path, confidence=50, overlap=30).json()
    # change from 80, 30
    # visualize your prediction
    model.predict(img_path, confidence=50, overlap=30).save(img_path)
    return prediction_data

# %%
def get_selected_points(x_min, x_max, z_min, z_max, grid, data):
    selected_colors = []
    selected_points = []
    # img_test = cv2.imread('predictions\prediction22.jpg')

    for x in range(x_min, x_max, 1):
        for z in range(z_min, z_max, 1):
            idx = grid[x, z]
            if idx != -1:
                selected_colors.append(data[idx][0])
                selected_points.append(data[idx][1])
            # img_test[x, z] = [255, 0, 0] #BRG

    return np.array(selected_points), np.array(selected_colors)

# %%
def read_pcd(path):
    pcd = o3d.io.read_point_cloud(path)

    visualizer(pcd)

    return pcd

# %%
def get_bounding_box(prediction_data):

    # 2D bounding box coordinates
    b_boxes = []

    for pred in prediction_data['predictions']:
        bounding_box = {
            'z': pred['x'],
            'x': pred['y'],
            'width': pred['width'],
            'height': pred['height'],
        }

        # Image dimensions (to scale 2D to 3D)
        # image_width = prediction_data['image']['width']
        # image_height = prediction_data['image']['height']

        # Calculate 3D coordinates (assuming z-coordinate is arbitrary)
        x_min = int(bounding_box['x'] - bounding_box['height']/2)
        z_min = int(bounding_box['z'] - bounding_box['width']/2)
        x_max = int(bounding_box['x']  + bounding_box['height']/2)
        z_max = int(bounding_box['z'] + bounding_box['width']/2)

        temp_bb = [x_min, x_max, z_min, z_max]
        b_boxes.append(temp_bb)

    return b_boxes


# %%
def algorithm(points, colors, shape1, shape2, resolution, reverse, axis_s, axes):
    check_grid = np.zeros((shape1+2*resolution, shape2+2*resolution), dtype=int) #They will set to 1 if that [x, y] grid is used; 0 -> false
    grid = np.ones((shape1+2*resolution, shape2+2*resolution)) * -1 #the actual grid which will have indexes mapped to a map
    img = np.ones((shape1+2*resolution, shape2+2*resolution, 3), dtype=np.uint8) * 170  # Gray out the background for better identification
    data = {} #{-1:[('RGB'), ('XYZ')], } #the data. key is index number saved into grid and values are the RGB and xyz data

    # axis_s is the axis along which we are sorting the points
    points_sort = points[np.argsort(points[:, axis_s])[::reverse]]
    colors_sort = colors[np.argsort(points[:, axis_s])[::reverse]]

    # Min and max values of the axes [X, Y, Z] (Doesn't depend on what axis we are doing it along)
    max_val = np.max(np.asarray(points_sort), axis=0)
    min_val = np.min(np.asarray(points_sort), axis=0)

    # The 2 coordinate axes for the 2d image
    axis1, axis2 = axes

    center1 = (min_val[axis1]+max_val[axis1])/2
    range1 = np.abs(min_val[axis1]) + np.abs(max_val[axis1])
    x_r_new = shape1 - 1

    center2 = (min_val[axis2]+max_val[axis2])/2
    range2 = np.abs(min_val[axis2]) + np.abs(max_val[axis2])
    z_r_new = shape2 - 1

    # For every single sorted point, iterate until you go to the end point
    for idx, val in enumerate(points_sort):
        # Conversion from 3D (but only two axes) to 2D

        # Scaling bt -ve inf. -> 1
        scale1 = ((val[axis1] - center1) / range1)
        scale2 = ((val[axis2] - center2) / range2)

        # print(scale1, scale2)
        index1 = int(scale1 * x_r_new + x_r_new/2) + resolution
        index2 = int(scale2 * z_r_new + z_r_new/2) + resolution

        if not check_grid[index1, index2]:

            check_grid[index1, index2] = 1
            data[idx] = [colors_sort[idx], val]
            grid[index1, index2] = idx
            col = np.uint8(colors_sort[idx]*255)
            img[index1, index2] = col
            # do around the point too of certain threshold

            dx_l = np.linspace(-resolution+index1, resolution+index1, 2*resolution+1, dtype=int)
            dy_l = np.linspace(-resolution+index2, resolution+index2, 2*resolution+1, dtype=int)

            dx, dy = np.meshgrid(dx_l, dy_l)

            temp_check_grid = check_grid[min(dx_l):max(dx_l)+1, min(dy_l):max(dy_l)+1]

            dx1 = (dx >= 0) & (dx < shape1) & ~temp_check_grid
            dy1 = (dy >= 0) & (dy < shape2) & ~temp_check_grid
            dx_dy = dx1 & dy1

            ones = np.argwhere(dx_dy)
            # scaling up
            ones[:, 0] += min(dx_l)
            ones[:, 1] += min(dy_l) #indices for check_grid

            # Set all the indices in check_grid to seen
            check_grid[ones[:, 0], ones[:, 1]] = 1
            img[ones[:, 0], ones[:, 1]] = col
    return check_grid, grid, img, data

# %%
def save_img(image, img_path):
    image = Image.fromarray(image)
    image.save(img_path)
    print("Saved!")

# %%
def get_bounding_box_2d(img_path, rect_threshold):
    thresholded_image = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    edged = cv2.Canny(thresholded_image, 50, 100)

    contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    bounding_boxes = []
    for contour in contours:
        # Get bounding box coordinates
        x, y, w, h = cv2.boundingRect(contour)
        if (w*h) > rect_threshold:
            bounding_boxes.append((x, y, w, h))

    original_image = cv2.imread(img_path)  # Load your original image here
    bounding_box_image = original_image.copy()
    for (x, y, w, h) in bounding_boxes:
        cv2.rectangle(bounding_box_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Draw green rectangles

    # Display or save the image with bounding boxes
    cv2.imwrite(img_path, bounding_box_image)

    # x, y is top left point, not center
    return bounding_boxes #[[x, y, w, h]]

# %%
def color_detection(img_path):
    image = cv2.imread(img_path)

    img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Color for the white color in the flowers

    lower_range = np.array([150, 160, 185])
    upper_range = np.array([360, 255, 255])

    mask = cv2.inRange(img_hsv, lower_range, upper_range)

    outside_range_mask = cv2.bitwise_not(mask)

    img_hsv[outside_range_mask != 0] = [0, 0, 255]

    result_image = cv2.cvtColor(img_hsv, cv2.COLOR_HSV2BGR)
    cv2.imwrite(img_path , result_image)

    bounding_boxes = get_bounding_box_2d(img_path, rect_threshold=2500)
    
    # x, y is top left point, not center
    # return the maxes and mins of both the axes
    axes_ranges = []
    for x, y, w, h in bounding_boxes: #[x, y, w, h]
        z_min = int(y)
        x_min = int(x)
        z_max = int(y + h)
        x_max = int(x + w)

        # print( x, y, w, h, x_min, x_max, z_min, z_max)

        axes_ranges.append([x_min, x_max, z_min, z_max])

    return axes_ranges

def initialization(pcd):
    # %%
    x_shape = 7000
    y_shape = 7000
    z_shape = 7000

    iterations = [
        {"shape": [y_shape, z_shape], "reverse": 1, "axis":0, "axes": [1, 2]}, #X
        {"shape": [x_shape, z_shape], "reverse": 1, "axis":1, "axes": [0, 2]}, #Y
        {"shape": [x_shape, y_shape], "reverse": 1, "axis":2, "axes": [0, 1]}, #Z
        {"shape": [y_shape, z_shape], "reverse": -1, "axis":0, "axes": [1, 2]}, #-X
        {"shape": [x_shape, z_shape], "reverse": -1, "axis":1, "axes": [0, 2]}, #-Y
        {"shape": [x_shape, y_shape], "reverse": -1, "axis":2, "axes": [0, 1]} #-Z
    ]

    # pcd = o3d.io.read_point_cloud("../3dmodels/Bell_Pepper_6_26.ply")
    # pcd = o3d.io.read_point_cloud("../3dmodels/Strawberry_White.ply")
    # pcd = o3d.io.read_point_cloud("../3dmodels/Data/Data 15 PG.ply")

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=pcd.get_center())

    visualizer([pcd, mesh_frame])

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    rf = Roboflow(api_key="ts8jw999gCMVw72LHUzT")
    # project = rf.workspace().project("strawberry-flower-detection-il6jq")
    project = rf.workspace().project("polinizador")
    model = project.version(1).model

    j = 0
    colors = [
        [255, 0, 0],   # Red
        [0, 255, 0],   # Green
        [0, 0, 255],   # Blue
        [255, 255, 0], # Yellow
        [0, 255, 255], # Cyan
        [0, 128, 0],   # Green (dark)
        [255, 255, 255] #black
    ]

    return iterations, pcd, model, j

# %%
def threshold_old():
    threshold = 2 # 0 -> 1

    x, y, z = np.asarray(pcd.points)[:, :3].T

    std = np.std((x, y, z), axis=1) * threshold

    hist_x, bins_x = np.histogram(x, bins=50)
    hist_y, bins_y = np.histogram(y, bins=50)

    max_freq_bin_x = bins_x[np.argmax(hist_x)]
    max_freq_bin_y = bins_y[np.argmax(hist_y)]

    min_bound = (max_freq_bin_x, max_freq_bin_y, -2) - std
    max_bound = (max_freq_bin_x, max_freq_bin_y, 2) + std

    bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound,
                                                        max_bound=max_bound)
    pcd = pcd.crop(bounding_box)

    visualizer(pcd)

# %%
def threshold(pcd):
    threshold = 25 # in % max, min: [1.9655249  2.17742805 1.19400877], [-1.96551809e+00 -2.17745111e+00  9.51918373e-04]
    center_pc = pcd.get_center()
    max_v = pcd.get_max_bound()
    min_v = pcd.get_min_bound()

    size = (max_v - min_v) * 0.01 * threshold

    min_bound = center_pc - size
    max_bound = center_pc + size
    min_bound[2] = -2
    max_bound[2] = 2
    bounding_box = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound,
                                                        max_bound=max_bound)

    pcd = pcd.crop(bounding_box)

    visualizer(pcd)

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    return points, colors


# %%
def main(iterations, points, colors, model, j):

    all_point_clouds = []
    all_points = []
    all_bounding_vals = []
    all_grid = []
    all_data1 = []
    show = False
    for i in iterations:

        print("In iter", j)
        img_path=f"numpy/img_{j}.jpg"
        start = time.time()
        check_grid, grid, img, data = algorithm(
            points=points,
            colors=colors,
            shape1=i["shape"][0],
            shape2=i["shape"][1],
            resolution=20,
            reverse=i["reverse"],
            axis_s=i["axis"],
            axes=i["axes"],)
        
        print("time taken: ")
        print((time.time() - start))
        save_img(image=img, img_path=img_path)
        print("image saved")

        method = "object" # or object

        if method == "object":
            predictions = object_detection(img_path=img_path, model=model)

            if len(predictions['predictions']) == 0:
                print("No objects found")
                j+=1
                continue

            bounding_boxes = get_bounding_box(prediction_data=predictions)

            temp_points = []
            temp_colors = []

        elif method == "color":
            bounding_boxes = color_detection(img_path=img_path)
            temp_points = []
            temp_colors = []

        else:
            j += 1
            continue

        for x_min, x_max, z_min, z_max in bounding_boxes:

            # selected_points, selected_colors = get_selected_points(z_min, z_max, x_min, x_max, grid, data)
            selected_points, selected_colors = get_selected_points(x_min, x_max, z_min, z_max, grid, data)

            if not selected_points.shape == (0,):
                if type(temp_points) == list:
                    temp_points = selected_points
                    temp_colors = selected_colors

                temp_points = np.concatenate([temp_points, selected_points], axis=0)
                temp_colors = np.concatenate([temp_colors, selected_colors], axis=0)

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(temp_points)
        point_cloud.colors = o3d.utility.Vector3dVector(temp_colors)
        # point_cloud.paint_uniform_color(color[j])

        all_point_clouds.append(point_cloud)

        print("All the combined point clouds")

        if show:
            visualizer([point_cloud])

        # o3d.io.write_point_cloud(f"pcd_{j}.ply", point_cloud)

        print("Done iter:", j)

        j += 1

    if show:
        print("Outside the loop")
        visualizer(all_point_clouds)

    pcd_final = o3d.geometry.PointCloud()

    for i in all_point_clouds:
        pcd_final += i

    visualizer(pcd_final)

    return all_point_clouds, pcd_final


# %%
def filter_green(hsv_color):

    low_reject_h = 60/360
    high_reject_h = 170/360
 
    low_reject_s = 0.2
    high_reject_s = 1.0
 
    low_reject_v = 0.0
    high_reject_v = 0.6
 
    if (low_reject_v <= hsv_color[2] <= high_reject_v):
        return True
    if (low_reject_h <= hsv_color[0] <= high_reject_h) and (low_reject_s <= hsv_color[1] <= high_reject_s):
        return True
    else:
        return False

# %%
def DBClustering(all_pcd, pcd):
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

    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(
            all_pcd.cluster_dbscan(eps=0.02, min_points=30))

    max_label = labels.max()
    outlier_cloud_sph_2 =[]
    bounding_boxes = []
    final_pcd = []
    l = 0
    pcd_bb = []
    all_segments = o3d.geometry.PointCloud()

    # Label -1 is all the points which didn't fit any of the clusters
    for i in range(0, max_label+1):
        indexes = np.argwhere(labels == i).flatten()
        new_pc = o3d.geometry.PointCloud()
        new_pc.points = o3d.utility.Vector3dVector(np.asarray(all_pcd.points)[indexes])
        new_pc.colors = o3d.utility.Vector3dVector(np.asarray(all_pcd.colors)[indexes])
        if l >= 19:
            l = 0
        # new_pc.paint_uniform_color(color[l])
        l+= 1
        # get the avg distance bt points and remove it if its above a certain threshold
        temp_dist = np.average(new_pc.compute_nearest_neighbor_distance())

        avg_color = np.average(new_pc.colors, axis=0)
        
        if len(np.asarray(new_pc.points)) != 0 and not filter_green(avg_color):
            outlier_cloud_sph_2.append(new_pc)

            # find bounding box
            bb = new_pc.get_oriented_bounding_box()
            bounding_boxes.append(bb)
            temp_pcd = pcd.crop(bb)
            all_segments += temp_pcd
            final_pcd.append(temp_pcd)
            pcd_bb.append(temp_pcd)
            pcd_bb.append(bb)
            # final_pcd.append(bb)
            # visualizer(new_pc)
    visualizer(outlier_cloud_sph_2)

    return bounding_boxes, all_segments, pcd_bb, final_pcd

# %%
def write_bb(plant_id, bounding_boxes):
    dat = []
    for b in bounding_boxes:
        # print(b.center, b.R, b.extent)
        # print("\n")
        dat.append([b.center, b.R[0], b.R[1], b.R[2], b.extent])
    # print(np.asarray(dat))
    np.savez(f'{plant_id}_arrays.npz', np.asarray(dat))

# %%
# def draw_3d_bounding_box_new(pcd, all_pcds):
#     croped_out_pcd = []
#     visualizer(all_pcd)
#     seg = 0
#     new_bb = []

#     for cld in all_pcds:

#         bbox = cld.get_oriented_bounding_box()
#         # both are the same
#         # bbox = cld.get_axis_aligned_bounding_box()
#         cld.paint_uniform_color((255, 0, 0))

#         visualizer([cld, pcd, bbox])
#         new_bb.append(bbox)

#         temp_pcd = pcd.crop(bbox)
#         croped_out_pcd.append(temp_pcd)

#         visualizer(temp_pcd)
#         o3d.io.write_point_cloud(f"segment0.ply", cld)
#         seg += 1
# %%
if __name__ == '__main__':

    directory_path = 'C:/Users/vmuriki3/Documents/test_code/3dmodels/Final Data'

    for f in os.listdir(directory_path):
        print(os.path.join(directory_path, f))
        pcd = o3d.io.read_point_cloud(os.path.join(directory_path, f))

        iterations, pcd, model, j = initialization(pcd)

        points, colors = threshold(pcd=pcd)

        all_point_clouds, pcd_final = main(iterations=iterations,
            points=points,
            colors=colors,
            model=model,
            j=j)

        bounding_boxes, all_segments, pcd_bb, final_pcd = DBClustering(
            all_pcd=pcd_final,
            pcd=pcd)

        write_bb(f[-6:-4], bounding_boxes)