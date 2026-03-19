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
import numpy as np
import heapq
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

cell = 5
ifcrater = 0.2
ifwhite = 200


def gray(image):
    # If image is RGB
    if len(image.shape) == 3:
        return (0.299 * image[:, :, 0] +
                0.587 * image[:, :, 1] +
                0.114 * image[:, :, 2]).astype(np.uint8) #converts the floats in to 0-255 integers and wraps if it overflows
    return image

def grid_maker(image):
    craters = gray(image) >= 200 #2D array of true, false depending on if the greyscale of the pixel is greater than white threshold
    h, w = craters.shape    #.shape gives tuple in this case the size of the image ex. (560, 560) pixels
    rows = h // cell    #floor diving the heights/width of the original grid by the cell ratio so it would be 1:5
    columns = w // cell

    grid = np.zeros((rows, columns), dtype=np.uint8) #dtype=np.uint8 will convert the data type to 8 bits since we only need to add 0s and 1s
    #np.zeros is a function that creates an array filled with zero, in this case a 2D array of rows*columns of 0s

    #Finding which cells are blocked
    for r in range(rows):
        for c in range(columns):
            chunk = craters[r * cell:(r + 1) * cell, c * cell:(c + 1) * cell] #finds the chunk corresponding to the image 0:5, 0:5

            #Nested for loop to check the fraction of white:Black
            cnt = 0
            for row in chunk:
                for val in row:
                    if val == True:
                        cnt += 1
            
            if cnt/chunk.size >= ifcrater:
                grid[r,c] = 1
    return grid

def manhattan_distance(x,y):
    #Manhattan value for pathfinding
    return abs(x[0] - y[0]) + abs(x[1] - y[1])

def free_pos(grid, pos):
    #Checks a radius around the initial position from left to right for a free cell if the initial start is a crater
    if grid[pos[0], pos[1]] == 0:
        return pos

    rows, col = grid.shape
    #starts at one cell from the initial position and increases as it loops
    for radius in range(1, max(rows, col)):
        #Checks top left to bottom right of the radius
        for r in range(max(0, pos[0] - radius), min(rows, pos[0] + radius + 1)):
            for c in range(max(0, pos[1] - radius), min(col, pos[1] + radius + 1)):
                #Min/max makes sure radius expands and avoids indexerror exception
                #once it finds a position returns
                if grid[r, c] == 0:
                    return (r, c)
    #if the loop fully runs (the whole grid is checked) then nothing is returned
    return None


def astar(grid, start_pos, end_pos):
    #Pathfinding method
    rows, col = grid.shape
    priority_queue = []
    #heapq creates a self-sorting array that puts the smallest value at the front remvoing the need to constantly sort
    #Priority is the best case value and is how A* works (score + amount of distance left) Smallest is priority
    heapq.heappush(priority_queue, (0, start_pos))

    #Recording the path we take
    initialpath = {}

    score = {start_pos:0} #tracks the moves we take format is {(x,y) , # of moves} #how far we have gone

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)] #valid directions, up,down,left,right

    while priority_queue:
        p, pos = heapq.heappop(priority_queue)

        if pos == end_pos:
            #checks if the position is equal to the end position
            final_path = []
            #Final path we take
            while pos in initialpath:
                #puts positions from path into final_path but it is in reverse
                #Basically it starts with the end, then traces back up by having it equal the previous cell, until it
                #reaches a value that is not equal to the previous pos meaning it is at the start
                final_path.append(pos)
                pos = initialpath[pos]

            
            final_path.append(start_pos) #once all positions are inputted, adds the final start position
            final_path.reverse() #reverses the list
            return final_path
        
        r, c = pos

        up = (r - 1, c)
        down = (r + 1, c)
        left = (r, c - 1)
        right = (r, c + 1)

        surround = [up, down, left, right] #these become coordinates of neighboring cells

        for x, y in surround: #neightbor row and neighbor column, so it will run four times, up,down,left,right
            if 0 <= x < rows and 0 <= y < col and grid[x, y] == 0: #checks if (x,y) is within the grid and then if it is a free space
                new_score = score[pos] + 1 #records the new position alongs with its new score

                if (x, y) not in score or new_score < score[(x, y)]: #first checks if the neighboring cell has already been recorded, then checks if the score is better/less than the same cell
                    score[(x, y)] = new_score #adds the score to the position of the neighboring cell

                    priority = new_score + manhattan_distance((x, y), end_pos)
                    heapq.heappush(priority_queue, (priority, (x, y))) #adds the new neighboring position to the priority queue with its score and position
                    initialpath[(x, y)] = pos #Adds the pos to the initialpath

    return None #If there is nothing in priority_queue meaning there is no path

def pathfinder(image):
    grid = grid_maker(image)
    start = free_pos(grid, (0, 0)) #top left corner
    end = free_pos(grid, (grid.shape[0] - 1, grid.shape[1] - 1)) #bottom right corner

    if start == None or end == None:
        print("Image is all white and no where to go")
        return None
    
    final_path = astar(grid, start, end)

    if final_path is None:
        print("No path found")
        return None    
    
    print("Path found")
    print("Grid path length:", len(final_path))
    return final_path


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
    while True:
        accel_x, accel_y, accel_z = accel_gyro.acceleration
        magnitude = (accel_x**2 + accel_y**2 + accel_z**2) ** 0.5
        #CHECKS IF READINGS ARE ABOVE THRESHOLD
        if magnitude > THRESHOLD:
            print("SHAKE")

            #PAUSE
            name = "Test"
            photo_name = img_gen(name)

            picam2.start()
            image = picam2.capture_image("main")
            path = pathfinder(np.array(image))
            print(path)
            image.save(photo_name)

            
            git_push()
            print("picture done")
            picam2.stop()
            #PUSH PHOTO TO GITHUB

            
            time.sleep(1)  # debounce
            break
        
        #PAUSE
def test_take_photo():
    print("Test Mode: Click 'Space' then 'Enter' to take a photo")
    while True:
        key = input()
        if key == " ":
            print("SPACE")
            name = "Test"
            photo_name = img_gen(name)

            picam2.start()
            time.sleep(1)
            image = picam2.capture_image("main")
            path = pathfinder(image)
            print(path)
            image.save(photo_name)

            
            git_push()
            print("picture done")
            picam2.stop()
            #PUSH PHOTO TO GITHUB

            
            time.sleep(1)  #debounce
        elif key.lower() == "q":
            print("Exiting test mode")
            break



def main():
    #take_photo()           #Comment for testing with movement
    test_take_photo()       #Comment for Testing with Keyboard Input


if __name__ == '__main__':
    main()