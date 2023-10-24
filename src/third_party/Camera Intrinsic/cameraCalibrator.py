import os
import cv2
import numpy as np

from tqdm import tqdm
from typing import List

def create_directories(video_filename: str) -> List[str]:
    calibration_folder = file.split('.')[-2]
    frames_path = calibration_folder+"\\frames"
    parameters_path = calibration_folder+"\params"

    if  os.path.exists(calibration_folder) == False:
        os.mkdir(calibration_folder)
    
    if  os.path.exists(frames_path) == False:
        os.mkdir(frames_path)

    if os.path.exists(parameters_path) == False:
        os.mkdir(parameters_path)
    
    return [calibration_folder, frames_path, parameters_path]

def extract_frames(directories: List[str], interval: int = 100, flip: bool = False) -> None:
    i = 1
    i_saved = 0
    stream = cv2.VideoCapture(file)
    if stream.isOpened():
        for i in tqdm(range(int(stream.get(cv2.CAP_PROP_FRAME_COUNT)))):
            ret, frame = stream.read()
            if (i - i_saved) == interval:
                i_saved = i
                if flip:
                    frame_saved = cv2.flip(cv2.flip(frame,0),1)
                else:
                    frame_saved = frame
                cv2.imwrite(f"{directories[1]}\\frame_{i}.jpg",frame_saved)
            else:
                pass
            i += 1
    stream.release()

def estimate_params(directories: List[str], checkerboard_size: List[int] = [6,8], save_params: bool = True) -> None:
    obj_points = []
    img_points = []

    objp = np.zeros((checkerboard_size[0]*checkerboard_size[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1,2)

    for f in tqdm([f for f in os.listdir(directories[1])]):
        img = cv2.imread(f"{directories[1]}\{f}")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK)

        if ret == True:
            obj_points.append(objp)
            img_points.append(corners)

    if (len(img_points) == len(obj_points)) and (len(img_points) > 0):
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, img.shape[:2], None, None)
    else:
        print("No points found")
        return None


    if ret == True and save_params == True:
        np.savetxt(f"{directories[2]}\{mtx}.csv",mtx,delimiter=",")
        np.savetxt(f"{directories[2]}\{dist}.csv",dist,delimiter=",")
        np.savetxt(f"{directories[2]}\{rvecs}.csv",rvecs,delimiter=",")
        np.savetxt(f"{directories[2]}\{tvecs}.csv",tvecs,delimiter=",")
    else:
        print("Something broke")
        return None

    return None

def display_frames(directories:list[str]):
    cv2.namedWindow("Extracted frames")
    cv2.setMouseCallback("Extracted frames",mouse_cb)

    for f in [f for f in os.listdir(directories[1])]:
        img = cv2.imread(f"{directories[1]}\{f}")
        cv2.imshow("Extracted frames",img)
        cv2.waitKey(0)

def mouse_cb(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked point: ({x},{y})")

if __name__ == "__main__":

    file = "validation.avi"

    if os.path.isfile(file) == False:
        print(f"No video named: {file} found")
        exit()

    else:
        directories = create_directories(file)
        extract_frames(directories, interval = 20, flip=True)
        display_frames(directories)
        estimate_params(directories, [6,8])
