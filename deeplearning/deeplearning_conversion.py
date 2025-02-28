from roboflow import Roboflow
import open3d as o3d
import numpy as np
import cv2
from PIL import Image
import time

def visualizer(cloud):

    if type(cloud) != list:
        cloud = [cloud]

    if True:
        # point cloud
        # center_of_mass = np.mean(np.asarray(cloud[0].points), axis=0)
    
        o3d.visualization.draw_geometries(cloud,
                                    zoom=0.57899999999999985,
                                    front=  [ 0.053781796277127751, 0.99511863815317547, 0.082743062484869914 ],
                                    up =  [ -0.99452345262671604, 0.045944145215534776, 0.093873519673282182 ],
                                    lookat=[ 0.14950467828195535, -0.21448131248991498, 0.63221199653621662 ])
    

def algorithm(points, colors, shape1, shape2, resolution, reverse, axis_s, axes):
    check_grid = np.zeros((shape1, shape2)) #They will set to 1 if that [x, y] grid is used; 0 -> false
    grid = np.zeros((shape1, shape2)) #the actual grid which will have indexes mapped to a map
    img = np.ones((shape1, shape2, 3), dtype=np.uint8) * 255
    data = {} #{-1:[('RGB'), ('XYZ')], } #the data. key is index number saved into grid and values are the RGB and xyz data

    points_sort = points[np.argsort(points[:, axis_s])[::reverse]]
    colors_sort = colors[np.argsort(points[:, axis_s])[::reverse]]
    
    # point_cloud = o3d.geometry.PointCloud()
    # point_cloud.points = o3d.utility.Vector3dVector(points_sort)
    # visualizer([point_cloud])

    max_val = np.max(np.asarray(points_sort), axis=0)
    min_val = np.min(np.asarray(points_sort), axis=0)

    first_a, snd_a = axes

    cent_x = (min_val[first_a]+max_val[first_a])/2
    range_x = np.abs(min_val[first_a]) + np.abs(max_val[first_a])
    x_r_new = shape1 - 1

    cent_z = (min_val[snd_a]+max_val[snd_a])/2
    range_z = np.abs(min_val[snd_a]) + np.abs(max_val[snd_a])
    z_r_new = shape1 - 1

    for idx, val in enumerate(points_sort):
        x_c = int(((val[first_a] - cent_x) / range_x) * x_r_new + x_r_new/2)
        z_c = int(((val[snd_a] - cent_z) / range_z) * z_r_new + z_r_new/2)

        if not check_grid[x_c, z_c]:

            check_grid[x_c, z_c] = 1
            data[idx] = [colors_sort[idx], val]
            grid[x_c, z_c] = idx
            col = np.uint8(colors_sort[idx]*255)
            img[x_c, z_c] = col
            # do around the point too

            for dx in range(-resolution, resolution, 1):
                for dz in range(-resolution, resolution, 1):

                    if 0 <= x_c + dx < shape1 and 0 <= z_c + dz < shape2:
                        if not check_grid[x_c + dx, z_c + dz]:
                            check_grid[x_c + dx, z_c + dz] = 1
                            img[x_c + dx, z_c + dz] = col
    
    return check_grid, grid, img, data


def save_img(image, img_path):
    image = Image.fromarray(image)
    image.save(img_path)
    print("Saved!")


def object_detection(img_path, model):

    prediction_data = model.predict(img_path, confidence=80, overlap=30).json()

    # visualize your prediction
    # model.predict(img_path, confidence=80, overlap=30).save("test_img")
    return prediction_data


def get_bounding_box(prediction_data):

    # 2D bounding box coordinates
    bounding_box = {
        'z': prediction_data['predictions'][0]['x'],
        'x': prediction_data['predictions'][0]['y'],
        'width': prediction_data['predictions'][0]['width'],
        'height': prediction_data['predictions'][0]['height'],
    }

    # Image dimensions (to scale 2D to 3D)
    # image_width = prediction_data['image']['width']
    # image_height = prediction_data['image']['height']

    # Calculate 3D coordinates (assuming z-coordinate is arbitrary)
    x_min = int(bounding_box['x'] - bounding_box['height']/2)
    z_min = int(bounding_box['z'] - bounding_box['width']/2)
    x_max = int(bounding_box['x']  + bounding_box['height']/2)
    z_max = int(bounding_box['z'] + bounding_box['width']/2)

    return x_min, x_max, z_min, z_max


def get_selected_points(x_min, x_max, z_min, z_max, grid, data):
    selected_colors = []
    selected_points = []
    # img_test = cv2.imread('predictions\prediction22.jpg')

    for x in range(x_min, x_max, 1):
        for z in range(z_min, z_max, 1):
            idx = grid[x, z]
            selected_colors.append(data[idx][0])
            selected_points.append(data[idx][1])
            # img_test[x, z] = [255, 0, 0] #BRG

    return np.array(selected_points), np.array(selected_colors)


def draw_3d_bounding_box(pcd, all_points):

    for cld in all_points:

        # bbox = cld.get_oriented_bounding_box()
        bbox = cld.get_axis_aligned_bounding_box()

        visualizer([pcd, bbox])

        pcd = pcd.crop(bbox)

        visualizer(pcd)

    o3d.io.write_point_cloud(f"final_pcd_bell_pepper.ply", pcd)

    visualizer([pcd])

    
def draw_one_bounding_box(pcd, all_points):

    all_point_cloud = o3d.geometry.PointCloud()

    for i in all_points:
        all_point_cloud += i

    bbox = all_point_cloud.get_axis_aligned_bounding_box()
    # for cld in all_points:

    #     # bbox = cld.get_oriented_bounding_box()
    #     bbox = cld.get_axis_aligned_bounding_box()

    #     visualizer([pcd, bbox])

    #     pcd = pcd.crop(bbox)

    #     visualizer(pcd)

    final = bbox+pcd

    o3d.io.write_point_cloud(f"final_pcd_bell_pepper_w_bb.ply", final)

    visualizer([pcd, bbox])

def read_pcd(path):
    pcd = o3d.io.read_point_cloud(path)

    visualizer(pcd)

    return pcd


def main():
    x_shape = 5000
    y_shape = 5000
    z_shape = 5000

    iterations = [
        {"shape": [y_shape, z_shape], "reverse": 1, "axis":0, "axes": [1, 2]}, #X
        {"shape": [x_shape, z_shape], "reverse": 1, "axis":1, "axes": [0, 2]}, #Y
        {"shape": [x_shape, y_shape], "reverse": 1, "axis":2, "axes": [0, 1]}, #Z
        {"shape": [y_shape, z_shape], "reverse": -1, "axis":0, "axes": [1, 2]}, #-X
        {"shape": [x_shape, z_shape], "reverse": -1, "axis":1, "axes": [0, 2]}, #-Y
        {"shape": [x_shape, y_shape], "reverse": -1, "axis":2, "axes": [0, 1]} #-Z
    ]

    # pcd = o3d.io.read_point_cloud("3dmodels/Bell_Pepper_6_26.ply")
    pcd = o3d.io.read_point_cloud("../3dmodels/Data/Data 11.ply")

    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=pcd.get_center())

    visualizer([pcd, mesh_frame])

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    # rf = Roboflow(api_key="p0DKMSH5Ym3FH9zh7ZLa")
    # project = rf.workspace().project("sweet-pepper-detection")
    rf = Roboflow(api_key="ts8jw999gCMVw72LHUzT")
    project = rf.workspace().project("strawberry-flower-detection-il6jq")
    model = project.version(1).model

    j = 0
    color = [
        [255, 0, 0],   # Red
        [0, 255, 0],   # Green
        [0, 0, 255],   # Blue
        [255, 255, 0], # Yellow
        [0, 255, 255], # Cyan
        [0, 128, 0],   # Green (dark)
    ]

    all_point_clouds = []
    all_points = []

    for i in iterations:

        print("In iter", j)
        img_path=f"numpy/img_{j}.jpg"
        check_grid, grid, img, data = algorithm(
            points=points,
            colors=colors,
            shape1=i["shape"][0],
            shape2=i["shape"][1],
            resolution=11,
            reverse=i["reverse"],
            axis_s=i["axis"],
            axes=i["axes"])
        
        save_img(image=img, img_path=img_path)

        predictions = object_detection(img_path=img_path, model=model)

        if len(predictions['predictions']) == 0:
            print("No objects found")
            j+=1
            continue

        x_min, x_max, z_min, z_max = get_bounding_box(prediction_data=predictions)

        selected_points, selected_colors = get_selected_points(x_min, x_max, z_min, z_max, grid, data)
        all_points.append(selected_points)

        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(selected_points)
        point_cloud.paint_uniform_color(color[j])

        all_point_clouds.append(point_cloud)

        visualizer([point_cloud, pcd])

        merged = point_cloud+pcd

        o3d.io.write_point_cloud(f"pcd_{j}.ply", point_cloud)

        print("Done ", j)

        j += 1

    visualizer(all_point_clouds)

    all_pcd = all_point_clouds[0]

    with open('output.txt', 'w') as f:
        for sublist in all_points:
            f.write(' '.join(map(str, sublist)) + '\n')

    draw_3d_bounding_box(pcd, all_point_clouds)


if __name__ == "__main__":
    main()
    # paths = ['pcd_2.ply', 'pcd_4.ply', 'pcd_5.ply']
    # pcd = o3d.io.read_point_cloud("3dmodels/Bell_Pepper_6_26.ply")

    # all_pcd = []

    # for path in paths:
    #     temp_pcd = read_pcd(path)

    #     all_pcd.append(temp_pcd)

    # draw_3d_bounding_box(pcd, all_pcd)