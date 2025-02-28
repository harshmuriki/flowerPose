import cv2
import os

def images_to_video(image_folder, output_path, fps):
    # Get the list of all files in directory
    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
    # images.sort()  # Sort the images by name (assumes names are in chronological order)

    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape

    video = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

    for image in images:
        video.write(cv2.imread(os.path.join(image_folder, image)))

    cv2.destroyAllWindows()
    video.release()

if __name__ == "__main__":
    image_folder = 'C:/Users/vmuriki3/Pictures/strawberry_white_robot_1'
    output_video = 'video1.mp4'
    frames_per_second = 15.0

    images_to_video(image_folder, output_video, frames_per_second)


# from time_lapse import output, source
# import os

# # Specify the folder containing your images
# image_folder = 'C:/Users/vmuriki3/Pictures/plant_15_handpicked/plant 15 handpicked/'

# # Get a list of all JPEG images in the folder
# image_files = [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith('.jpg')]

# # Use time_lapse library to create the timelapse output
# source_input = source.get_input(image_files, fps=24, deflicker=10, filters=None)
# output.create_outputs(source_input, 'test_movie', verbose=False)
