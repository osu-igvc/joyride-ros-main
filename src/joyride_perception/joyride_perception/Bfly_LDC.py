# Pathing
import os
from PIL import Image

# Utilities
import cv2 as cv
import glob
import numpy as np

# Camera requirements
import PySpin
from simple_pyspin import Camera

class Bfly_LDC():
    def __init__(self):
        """
        Description:
        Initializes the Bfly_LDC (BlackFly Lens Distortion Calibration) class, checking for the initial pathway needed for calibration.

        args:
            None  

        Returns:
            None
        """


        # Check/Make a directory to save Data
        self.output_dir = 'Camera Calibration Data'
        print(f'Checking for {self.output_dir} folder.')
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            print(f'No folder for {self.output_dir}. Creating folder.')
        else:
            print(f'Folder for {self.output_dir} located.')

        # Aquiring aquisition time
        while True:
            try:
                self.duration = int(input("Enter the calibration duration in seconds: "))
                print(f"Starting each camera for: {self.duration} seconds" )
                break
            except ValueError:
                print("Invalid input. Please enter an integer.")
                continue


    def getCameraImages(self):
        """
        Checks all connected cameras and corresponding files before asking to open
        a new instance for each camera and recording video for calibration.

        args:
            None  

        Returns:
            None
        """
        try:
             # Create a System object
            system = PySpin.System.GetInstance()

            # Get a list of cameras from the System object
            cam_list = system.GetCameras()

            num_cameras = cam_list.GetSize()

            print('Number of cameras detected: %d' % num_cameras)

            # Finish if there are no cameras
            if num_cameras is 0:
                print('No Cameras Found!')
                
            else:
                for i, cams in enumerate(cam_list):

                    # Retrieve device serial number for filename
                    node_device_serial_number = PySpin.CStringPtr(cams.GetTLDeviceNodeMap().GetNode('DeviceSerialNumber'))

                    # Holds Camera path to save data 
                    cam_dir_path = ''

                    #  Checks serial number
                    if PySpin.IsReadable(node_device_serial_number):
                        device_serial_number = node_device_serial_number.GetValue()
                        print('Camera %d serial number set to %s...' % (i, device_serial_number))

                        #Assigns name to camera. To add extra cameras add "case '<Serial Number>:" followed by "cam_name = '<Camera Name>'
                        # TODO Get the correct serial numbers
                        cam_name = ''
                        match device_serial_number:
                            case '18295825':
                                cam_name = 'BlackFly Left'
                            case '18295818':
                                cam_name = 'BlackFly Right'
                            case '18295827':
                                cam_name = 'BlackFly Center'
                            case _ :
                                cam_name = 'Unkown Camera'

                        # === Creating path for camera data to be save === #
                        # Checks if camera data path exits if not makes one.
                        print('Checking Files'.center(25, '='))

                        cam_dir_path = os.path.join(self.output_dir,cam_name)
                        print(f'Checking for specific camera "{cam_dir_path}" folder.')
                        if not os.path.exists(cam_dir_path):
                            print(f'No folder for {cam_name} in path {self.output_dir}. Creating folder.')
                            os.makedirs(cam_dir_path)
                        else:
                            print(f'Folder for {cam_name} in path {cam_dir_path} located.')

                        # Checks if camera image path exits if not makes one.
                        cam_image_dir_path = os.path.join(cam_dir_path, 'Images')
                        print(f'Checking for specific camera "{cam_image_dir_path}" folder.')
                        if not os.path.exists(cam_image_dir_path):
                            print(f'No folder for "Images" in path {cam_image_dir_path}. Creating folder.')
                            os.makedirs(cam_image_dir_path)
                        else:
                            print(f'Folder for "Images" in path {cam_dir_path} located.')
                        print('File Check Completed'.center(25, '='))
                        # === End of file confirmation === #

                    # Input validation for camera to be calibrated
                    while True:
                        
                        print(f'Do you wish to Calibrate Distortion Matrix for {cam_name}')
                        user_input = input('Confirm? [Y/N]: ')
                        
                        if user_input.lower() in ('y', 'yes', 'Y', 'Yes', 'YES'):
                            # Acquire and initialize Camera
                            print(f'Opening camera {cam_name}')
                            with Camera(device_serial_number) as cam: 

                                
                                camWidth  = cam.SensorWidth
                                camHeight = cam.SensorHeight

                                print(f'Camera {device_serial_number} is {camWidth} by {camHeight} pixels')

                                # Allows color from from image 
                                if 'Bayer' in cam.PixelFormat:
                                    cam.PixelFormat = "RGB8Packed"

                                print("Image format in: " + cam.PixelFormat)

                                cam.start() # Start recording
                                imgs = [cam.get_array() for n in range(self.duration)] # Get frames

                                cam.stop() # Stop recording
                                print("Ending recording")

                            print("Saving calibration images to: %s" % cam_image_dir_path)

                            # cycles through all the frames until they are all saved in the corresponding cameras folder
                            for n, img in enumerate(imgs):
                                Image.fromarray(img).save(os.path.join(cam_image_dir_path, '%08d.png' % n))

                            # Begining Calibration
                            self.calibrateDistortion(cam_dir_path,cam_image_dir_path, camWidth, camHeight)
                            break
                            

                        elif user_input.lower() in ('n', 'no', 'N', 'No','NO'):
                            print(f'Skipping Calibration for: {cam_name}')
                            break

                        else:
                            # ... error handling ...
                            print(f'Error: Input {user_input} unrecognised.')
                            pass
                        
                # The usage of del is preferred to assigning the variable to None for the cameras.
                del cams

            #NOTE: If you break a loop without terminating the cameras it will lead to a sudden forced quit and not proceed
            # to the next section after this method is run
            

            # Clear camera list before releasing system
            cam_list.Clear()

            # Release system instance
            system.ReleaseInstance()
        
        
        except Exception as e:
            print('Get Image Error'.center(100, '='))
            print(e)
        
        




    def calibrateDistortion(self,data_path, image_path, camWidth, camHeight):
        """
        Calibrates lens distortion using PNG images from the specified filepath and saves the calibration data into a numpy array for later use.

        Args:
            data_path (str): The filepath containing PNG images used for calibration.
            image_path (str): The path to the directory where the calibrated images will be saved.
            camWidth (int): The width of the camera frame.
            camHeight (int): The height of the camera frame.

        Returns:
            None

        Saves:
            Calibration data in numpy arrays (Ret.npy, Mtx.npy, Dist.npy, Rvecs.npy, Tvecs.npy, NewCameraMtx.npy).
        """
        _ , camera_name = os.path.split(data_path)
        print(f'Beginning Calibration for {camera_name}'.center(50,'='))
        try:     
            # termination criteria
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            # prepare object points to look for on Chessboard
            cbrow = 7 # Chessboard rows
            cbcol = 6 # Chessboard columns

            print(f'looking for {cbrow} by {cbcol} board')

            objp = np.zeros((cbrow * cbcol, 3), np.float32)
            objp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)

            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.

            images = glob.glob(f'{image_path}\*.png')

            for fname in images:

                calImg = cv.imread(fname)
                gray = cv.cvtColor(calImg, cv.COLOR_BGR2GRAY)

                # Find the chess board corners
                ret, corners = cv.findChessboardCorners(gray, (cbrow,cbcol), None)

                # If found, add object points, image points (after refining them)
                if ret == True:
                    objpoints.append(objp)
                    corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                    imgpoints.append(corners2)

                    # [[[ NOT REQUIRED ]]]
                    # Draw and display the corners 
                    # cv.drawChessboardCorners(calImg, (cbrow,cbcol), corners2, ret)
                    # cv.imshow('img', calImg) # Ubuntu has issues displaying images should still run past this step
                    # cv.waitKey(1)

                else:
                    print(f'No Calibration board found in: {fname}. Deleting image.')
                    os.remove(fname)


            cv.destroyAllWindows()

            # Gathers callibration data
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (camWidth,camHeight), 1, (camWidth,camHeight))

            #TODO undistort and send back or copy into camera drives
            # currently ony used here on images to get error total

            # mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (camWidth,camHeight), 5)
            # dst = cv.remap(calImg, mapx, mapy, cv.INTER_LINEAR)


            # cv.imshow('img', calImg) # Ubuntu has issues displaying images should still run past this step
            # cv.waitKey(0)

            # mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (camWidth,camHeight), 5)
            # dst = cv.remap(calImg, mapx, mapy, cv.INTER_LINEAR)
            # x, y, w, h = roi
            # dst = dst[y:y+h, x:x+w]
            # print(os.path.join(data_path,'calibresult.png'))
            # cv.imwrite(os.path.join(data_path,'calibresult.png'), dst)

            # Checks and displays total error of distortion
            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
                mean_error += error
            print( "total error: {}".format(mean_error/len(objpoints)) )

            
            # Input validation
            while True:
                print(f'Do you wish so Save and Overide the Calibration Distortion Matrix for {camera_name}')
                user_input = input('Confirm? [Y/N]: ')
                
                if user_input.lower() in ('y', 'yes', 'Y', 'Yes', 'YES'):
                    # Converting to numpy array for easy save/load data
                    ret_numpy          = np.array(ret)
                    mtx_numpy          = np.array(mtx)
                    dist_numpy         = np.array(dist)
                    rvecs_numpy        = np.array(rvecs)
                    tvecs_numpy        = np.array(tvecs)
                    newcameramtx_numpy = np.array(newcameramtx)

                    with open(os.path.join(data_path,'Ret.npy'), 'wb') as f:
                        np.save(f,ret_numpy)
                    with open(os.path.join(data_path,'Mtx.npy'), 'wb') as f:
                        np.save(f,mtx_numpy)
                    with open(os.path.join(data_path,'Dist.npy'), 'wb') as f:
                        np.save(f,dist_numpy)
                    with open(os.path.join(data_path,'Rvecs.npy'), 'wb') as f:
                        np.save(f,rvecs_numpy)
                    with open(os.path.join(data_path,'Tvecs.npy'), 'wb') as f:
                        np.save(f,tvecs_numpy)
                    with open(os.path.join(data_path,'NewCameraMtx.npy'), 'wb') as f:
                        np.save(f,newcameramtx_numpy)
                    
                    print(f'New Calibration Data has been saved for {camera_name}.')
                    break


                elif user_input.lower() in ('n', 'no', 'N', 'No','NO'):
                    print(f'New Calibration Data has __NOT__ been saved for {camera_name}.')
                    break


                else:
                # ... error handling ...
                    print(f'Error: Input {user_input} unrecognised.')
                    continue

        # ... More Error Handeling ...
        except IndexError:
            print('INDEX ERROR'.center(100, '-'))
        except Exception as e: 
            print('CALIBRATION ERROR'.center(100, '-'))
            print(e)



def main():
    print(' Start of Bfly_LDC '.center(100, '+'))

    calibrateCameras = Bfly_LDC()
    calibrateCameras.getCameraImages()

    #NOTE: If this doesnt appear then cams werent closed when break occurs check getcameraImages
    print(' End of Bfly_LDC '.center(100, '+'))

if __name__ == '__main__':
    main()

