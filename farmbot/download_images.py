import json
import requests
from acquire_token import get_headers
# from print_settings import get_firmware_config, get_axis_length
import os
import time
import wget
import threading

def download_image(url, folder, filename):
    print("file1", filename)
    full_path = os.path.join(folder, filename)
    wget.download(url, full_path)

def get_path(plant_id, name):
    folder_name = "{:s}-{:d}".format(name, plant_id)
    if folder_name not in os.listdir("timelapses"):
        os.mkdir(os.path.join("timelapses", folder_name))
    return os.path.join("timelapses", folder_name)

response = requests.get('https://my.farmbot.io/api/images', headers=get_headers())
images = response.json()

# print(images)

i = 0
threads = []
folder = "test1"

if not os.path.exists(folder):
    os.makedirs(folder)

for image in images:
    filename = "img_{:}.jpg".format(str(i))
    print(filename)
    url = image["attachment_url"]

    t1 = threading.Thread(target=download_image, args=(url, folder, filename))
    t1.start()
    threads.append(t1)

    # save_image_only(url, filename)
    i+= 1

for thread in threads:
    thread.join()

