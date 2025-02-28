import cv2
import os
import subprocess
import numpy as np
import time
import sys
# from moviepy.video.io.VideoFileClip import VideoFileClip
from moviepy.editor import VideoFileClip, concatenate_videoclips
import shutil

def convert_to_photos(ipt_video, opt_folder):
    # Path to your input video file
    input_video_path = ipt_video

    # Directory where extracted frames will be saved
    output_dir = opt_folder

    # Open the video file
    cap = cv2.VideoCapture(input_video_path)

    # Check if the video file was successfully opened
    if not cap.isOpened():
        print("Error opening video file")
        exit()

    # Get the frame rate (fps) of the video
    fps = cap.get(cv2.CAP_PROP_FPS)

    # Create the output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Initialize variables
    frame_count = 0
    success = True

    val = 0
    # Read and save frames from the video
    while success:
        success, frame = cap.read()  # Read a frame

        if not success:
            break

        # Save the frame as an image file
        frame_count += 1
        frame_name = f"frame_{frame_count:04d}.jpg"  # Frame name format: frame_0001.jpg, frame_0002.jpg, ...
        frame_path = os.path.join(output_dir, frame_name)
        cv2.imwrite(frame_path, frame)  # Save frame as JPEG
        if frame_count % 1000 == 0:
            # Print progress
            print(f"Extracted frame {frame_count}")

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

    print("Frames extraction completed.")

def crop_video_cv(input_video_path, output_video_path):
    cap = cv2.VideoCapture(input_video_path)

    # Check if the video file was successfully opened
    if not cap.isOpened():
        print("Error opening video file")
        exit()

    # Get the frame rate (fps) and total number of frames
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Calculate the number of frames in the first 10 minutes
    ten_minutes_frames = int(fps * 60 * 14)

    # Create VideoWriter object to save the first 10 minutes of video
    # folder = 'images_15'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path, fourcc, fps, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    # Read and write frames for the first 10 minutes
    frame_count = 0
    temp_cnt = 0
    cnt = 0
    while frame_count < total_frames:
        ret, frame = cap.read()  # Read a frame

        if not ret:
            break

        if frame_count%15 == 0:
            out.write(frame)  # Write the frame to the output video file
            cv2.imwrite("images_15/{}.jpg".format(frame_count), frame)
            temp_cnt = 0
            cnt += 1

        # out.write(frame)  # Write the frame to the output video file
        frame_count += 1
        temp_cnt += 1

    # Release resources
    cap.release()
    out.release()

    print("Video extraction completed.")
    print(cnt, frame_count)
 
def getBestIndexFromImageList(imgList):
 
    max_index = -1
    max_joint_score = -sys.maxsize - 1
 
    for index, img in enumerate(imgList):
 
        # Convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
 
        # Get average brightness
        brightness = np.mean(v)
 
        # Get focus score
        y, x, _ = img.shape
        cx, cy = int(x/2), int(y/2)
        size = 200
        new_frame = img[cy-size:cy+size, cx-size:cx+size]
        gray_img = cv2.cvtColor(new_frame, cv2.COLOR_BGR2GRAY)
        focus_score = cv2.Laplacian(gray_img, cv2.CV_64F).var()
 
        # EDIT WEIGHTS HERE
        alpha = 2.5
        beta = 2.5
        joint_score = alpha*focus_score - beta*abs(brightness - 105)
 
        if (joint_score > max_joint_score):
            max_joint_score = joint_score
            max_index = index
        # print(index, joint_score)
    return max_index

def get_boxes(ipt_path, opt):
    cap = cv2.VideoCapture(ipt_path)

    # Check if the video file was successfully opened
    if not cap.isOpened():
        print("Error opening video file")
        exit()

    # Get the frame rate (fps) and total number of frames
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    # Need 200 images
    boxes = int(np.floor(total_frames/200))
    print("total frames:", total_frames)
    print(boxes)

    # Calculate the number of frames in the first 10 minutes
    # ten_minutes_frames = int(fps * 60 * 5)

    # Create VideoWriter object to save the first 10 minutes of video
    output_video_path = '../data/best_auto/' + opt + '/'
    print(output_video_path)

    if not os.path.exists(output_video_path):
        os.makedirs(output_video_path)
        print(f"Folder created.")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_video_path+'video.mp4', fourcc, fps, (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))))

    # Read and write frames for the first 10 minutes
    frame_count = 1
    temp_cnt = 0
    cnt = 0
    temp_arr = [] 
    print("Processing")
    while frame_count < total_frames:
        ret, frame = cap.read()  # Read a frame
        temp_arr.append(frame)

        if not ret:
            break

        if frame_count % boxes == 0:
            # print(len(temp_arr))
            idx = getBestIndexFromImageList(temp_arr)
            temp_f = temp_arr[idx]
            out.write(temp_f)  # Write the frame to the output video file
            cv2.imwrite(output_video_path + f'{frame_count}.jpg', temp_f)
            temp_cnt = 0
            cnt += 1
            temp_arr = []
            print(frame_count)
            # break

        # out.write(frame)  # Write the frame to the output video file
        frame_count += 1
        temp_cnt += 1

    # Release resources
    cap.release()
    out.release()

    print("Video extraction completed.")
    print(cnt, frame_count)

def combine_videos(video_files, output_file):

    # cap = cv2.VideoCapture(video_files[0])
    # width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    # cap.release()

    # # Define the codec and create VideoWriter object
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Be sure to use lower case
    # out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))
    # print("Starting")

    # # Iterate through each video file and write to output
    # for video_file in video_files:
    #     cap = cv2.VideoCapture(video_file)
    #     while True:
    #         ret, frame = cap.read()
    #         if not ret:
    #             break
    #         out.write(frame)
    #     cap.release()

    # out.release()
    # print(f"Merged {len(video_files)} videos into {output_file}")
    
    if len(video_files) == 1:
        shutil.copy(video_files[0], output_file)
    else:
        videofiles = []

        for video in video_files:
            videofiles.append(VideoFileClip(video))

        # Concatenate the video clips
        final_clip = concatenate_videoclips(videofiles)

        # Write the concatenated video to a file
        final_clip.write_videofile(output_file, codec='libx264', fps=final_clip.fps)

        # Close the video clips
        for video in videofiles:
            video.close()

    print("Videos merged")


def crop_video(input_file, output_file, start_time, end_time):
    # Load the video clip
    clip = VideoFileClip(input_file)
    
    # Calculate the duration to crop
    duration = end_time - start_time
    print("Starting")
    
    # Crop the video clip
    cropped_clip = clip.subclip(start_time, end_time)
    
    # Save the cropped video clip to a file
    cropped_clip.write_videofile(output_file, codec='libx264', fps=clip.fps)
    
    # Close the original video clip
    clip.close()

def pipeline(folder):

    directory = f"C:/Users/vmuriki3/Documents/test_code/data/data_{folder}"
    all_videos = []

    for root, directories, files in os.walk(directory):
        for filename in files:
            # Join the root path and the file name to get the full file path
            if filename.endswith('.mp4'):
                file_path = os.path.join(root, filename)
                all_videos.append(file_path)

    print(all_videos)
    
    print("Combining Videos")
    combine_videos(all_videos, f"C:/Users/vmuriki3/Documents/test_code/data/data_{folder}/merged.mp4")
    
    print("Choosing best 200 images")
    get_boxes(ipt_path=f"C:/Users/vmuriki3/Documents/test_code/data/data_{folder}/merged.mp4",
              opt=f'data_{folder}')

    print("Converting to images")
    convert_to_photos(ipt_video=f'C:/Users/vmuriki3/Documents/test_code/data/data_{folder}/merged.mp4',
                      opt_folder=f'C:/Users/vmuriki3/Documents/test_code/data/data_{folder}/images')

    print("Pipeline Done")
if __name__ == '__main__':
    pipeline(folder=16)

    # input_file = 'C:/Users/vmuriki3/Documents/test_code/data/data_16/video16_2.mp4'
    # output_file = 'C:/Users/vmuriki3/Documents/test_code/data/data_16/cropped16_2.mp4'
    # start_time = 0  # Start time in seconds
    # end_time = 740    # End time in seconds

    # crop_video(input_file, output_file, start_time, end_time)
