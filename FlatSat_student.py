"""
The Python code you will write for this module should read
acceleration data from the IMU. When a reading comes in that surpasses
an acceleration threshold (indicating a shake), your Pi should pause,
trigger the camera to take a picture, then save the image with a
descriptive filename. You may use GitHub to upload your images automatically,
but for this activity it is not required.

The provided functions are only for reference, you do not need to use them. 
You will need to complete the take_photo() function and configure the VARIABLES section
"""

#AUTHOR: 
#DATE:

#import libraries
import time
import board
import cv2
import numpy as np
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX as LSM6DS
from adafruit_lis3mdl import LIS3MDL
from git import Repo
from picamera2 import Picamera2

#VARIABLES
THRESHOLD = 4      #Any desired value from the accelerometer
REPO_PATH = "/home/pi/Orbitmind"     #Your github repo path: ex. /home/pi/FlatSatChallenge
FOLDER_PATH = "/Images"   #Your image folder path in your GitHub repo: ex. /Images

#imu and camera initialization
i2c = board.I2C()
accel_gyro = LSM6DS(i2c)
mag = LIS3MDL(i2c)
picam2 = Picamera2()


def git_push():
    """
    This function is complete. Stages, commits, and pushes new images to your GitHub repo.
    """
    try:
        repo = Repo(REPO_PATH)
        origin = repo.remote('origin')
        print('added remote')
        origin.pull()
        print('pulled changes')
        repo.git.add(REPO_PATH + FOLDER_PATH)
        repo.index.commit('New Photo')
        print('made the commit')
        origin.push()
        print('pushed changes')
    except:
        print('Couldn\'t upload to git')


def img_gen(name):
    """
    This function is complete. Generates a new image name.

    Parameters:
        name (str): your name ex. MasonM
    """
    t = time.strftime("_%H%M%S")
    imgname = (f'{REPO_PATH}/{FOLDER_PATH}/{name}{t}.jpg')
    return imgname


def take_photo():
    """
    This function is NOT complete. Takes a photo when the FlatSat is shaken.
    Replace psuedocode with your own code.
    """
    picam2.start()
    while True:
        accel_x, accel_y, accel_z = accel_gyro.acceleration
        magnitude = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5
        #CHECKS IF READINGS ARE ABOVE THRESHOLD
        if magnitude > THRESHOLD:
            print("SHAKE")

            #PAUSE
            name = "Test"
            photo_name = img_gen(name)


            image = picam2.capture_image("main")
            image_array = picam2.capture_array("main")


            
            image.save(photo_name)
            print("picture done")
            picam2.stop()
            #PUSH PHOTO TO GITHUB
            time.sleep(1)  # debounce
            return image_array, photo_name
        
        #PAUSE
    picam2.stop()


def main():
    image_arr, photo_name = take_photo()
    hsv(image_arr, photo_name)
    git_push()

def hsv(image, path):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    height, width, _ = hsv_image.shape
    quad_viz_hsv = np.zeros((height, width, 3), dtype=np.uint8)
    cy, cx = height // 2, width // 2
    slices = {
        "Top-Left":     (slice(0, cy), slice(0, cx)),
        "Top-Right":    (slice(0, cy), slice(cx, width)),
        "Bottom-Left":  (slice(cy, height), slice(0, cx)),
        "Bottom-Right": (slice(cy, height), slice(cx, width))
    }
    for name, (slice_y, slice_x) in slices.items():
        quad_data = hsv_image[slice_y, slice_x]
        #gets data from one quadrant/slice

        avg_hsv = np.mean(quad_data, axis=(0, 1))
        h, s, v = avg_hsv
        hue_degrees = h * 2
        sat_percent = (s / 255) * 100
        val_percent = (v / 255) * 100
        print(f"{name}:")
        print(f"Color: {hue_degrees:.1f}°, Saturation: {sat_percent:.1f}%, Brightness: {val_percent:.1f}%")
        print("-" * 30)

        avg_hsv_int = avg_hsv.astype(np.uint8)
        quad_viz_hsv[slice_y, slice_x] = avg_hsv_int

        quad_viz_bgr = cv2.cvtColor(quad_viz_hsv, cv2.COLOR_HSV2BGR)
        quad_path = path.replace(".jpg", "_quads.jpg")
        cv2.imwrite(quad_path, quad_viz_bgr)
    


if __name__ == '__main__':
    main()