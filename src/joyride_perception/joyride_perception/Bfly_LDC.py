# Pathing
import enum
import os
from PIL import Image
import glob

# Utilities
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

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
        self.dir_path = os.path.dirname(os.path.abspath(__file__))
        self.output_dir = 'Camera Calibration Data'
        self.duration = 0
        
        print(f'Checking for {self.output_dir} folder.')
        self.check_for_folder(self.output_dir)
            


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
            if num_cameras == 0:
                print('No Cameras Found!')
                raise UnboundLocalError()
                
                

            for i, cams in enumerate(cam_list):

                # Retrieve device serial number for filename
                node_device_serial_number = PySpin.CStringPtr(cams.GetTLDeviceNodeMap().GetNode('DeviceSerialNumber'))

                # Holds Camera path to save data 
                cam_dir_path = ''

                #  Checks serial number
                if PySpin.IsReadable(node_device_serial_number):
                    device_serial_number = node_device_serial_number.GetValue()
                    

                    #Assigns name to camera. To add extra cameras add "case '<Serial Number>:" followed by "cam_name = '<Camera Name>'
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
                    print('Camera %s serial number set to %s...' % (cam_name, device_serial_number))

                    # === Creating path for camera data to be save === #
                    # Checks if camera data path exits if not makes one.
                    print('Checking Files'.center(50, '='))

                    cam_dir_path = os.path.join(self.output_dir,cam_name)
                    print(f'Checking for specific camera "{cam_dir_path}" folder.')
                    self.check_for_folder(cam_dir_path)

                    # Checks if camera image path exits if not makes one.
                    cam_image_path = os.path.join(cam_dir_path, 'Images')
                    print(f'Checking for specific camera "{cam_image_path}" folder.')
                    self.check_for_folder(cam_image_path)

                    cam_array_path = os.path.join(cam_dir_path, 'Arrays')
                    print(f'Checking for specific camera "{cam_array_path}" folder.')
                    self.check_for_folder(cam_array_path)

                    cam_video_path = os.path.join(cam_dir_path, 'Videos')
                    print(f'Checking for specific camera "{cam_video_path}" folder.')
                    self.check_for_folder(cam_video_path)
                    
                    print('File Check Completed'.center(50, '='))
                    # === End of file confirmation === #

                # Input validation for camera to be calibrated
                    
                print(f'\nDo you wish to Calibrate Distortion Matrix for {cam_name}')        
                if not self.conformation(f'Calibrating Distortion Matrix for {cam_name}'):
                    continue
                
                # Aquiring aquisition time
                while True:
                    try:
                        self.duration = int(input("Enter the calibration duration in seconds: "))
                        print(f"Starting each camera for: {self.duration} seconds" )
                        self.duration = self.duration * 35
                        break
                    except ValueError:
                        print("Invalid input. Please enter an integer.")
                        continue
                
                print(f'Opening camera {cam_name}')
                with Camera(device_serial_number) as cam: 

                    camWidth  = cam.SensorWidth
                    camHeight = cam.SensorHeight

                    print(f'Camera {cam_name} is {camWidth} by {camHeight} pixels')

                    # Allows color from from image 
                    if 'Bayer' in cam.PixelFormat:
                        cam.PixelFormat = "RGB8Packed"

                    print("Image format in: " + cam.PixelFormat)

                    cam.start() # Start recording
                    imgs = [cam.get_array() for n in range(self.duration)] # Get frames

                    cam.stop() # Stop recording
                    print("Ending recording")

                # cycles through all the frames until they are all saved in the corresponding cameras folder
                self.record_video(filepath = cam_video_path,frames = imgs, width = camWidth, height = camHeight)
                
                print("Saving calibration images to: %s" % cam_image_path)
                for n, img in enumerate(tqdm(imgs)):
                    Image.fromarray(img).save(os.path.join(os.path.join(self.dir_path, cam_image_path), '%08d.png' % n))

                # Begining Calibration
                
                self.calibrateDistortion(cam_name, cam_image_path, cam_array_path, camWidth, camHeight)
                break   
                    
                        
            # The usage of del is preferred to assigning the variable to None for the cameras.
            del cams    

            #NOTE: If you break a loop without terminating the cameras it will lead to a sudden forced quit and not proceed
            # to the next section after this method is run
            

            # Clear camera list before releasing system
            cam_list.Clear()

            # Release system instance
            system.ReleaseInstance()
        
        except UnboundLocalError:
            pass
        
        except Exception as e:
            print('Get Image Error'.center(100, '='))
            print(e)
        

        # === Testing methods to clean up readability === #

    def conformation(self, message = 'Confirmed'):
        """
        Asks user for yes or no input after that is given returns a true or false

        args(str):
            Asks for confirmation message in a string format to be printed afer giving a [Y/N]

        returns(bool):
            Returns True if input was yes and returns False for no
        """
        user_input = input('Confirm? [Y/N]: ')
        while True:
            if user_input.lower() in ('y', 'yes', 'Y', 'Yes', 'YES'):
                print(f'{message}: {True}')
                return True
            elif user_input.lower() in ('n', 'no', 'N', 'No','NO'):
            
                print(f'{message}: {False}')
                break

            else:
            # ... error handling ...
                print(f'Error: Input {user_input} unrecognised.')
                pass        
        
        return False

    def check_for_folder(self,filename):
        """
        Checks for folder with given filename and creates one if one doesnt exists in the folder in which this script is contained 
        I.E. ( ../somefolder/Bfly_LDC) would create X folder (../somefolder/X)
        
        args: 
            filename - string containing filename or path in relation to this python script 
        
        """
        # Check/Make a directory to save Data
        print(f'PATH >>> {self.dir_path}')
        
        
        if not os.path.exists( os.path.join(self.dir_path, filename) ):
            os.makedirs(os.path.join(self.dir_path, filename))
            print(f'No folder for {filename} in path. Creating folder.\n')
                
        else:
            print(f'Folder for {filename} located in path.\n')
    

    def record_video(self, filepath, frames, width, height, fps = None):

        fps = 35

        fourcc = cv.VideoWriter_fourcc(*"XVID")
        #Syntax: cv2.VideoWriter( filename, fourcc, fps, frameSize )
        video = cv.VideoWriter(os.path.join(self.dir_path,filepath + '\calibration video.avi'), fourcc, float(fps), (width, height))
 
        for frame in enumerate(tqdm(frames)):
            video.write(frame[1])
 
        video.release()
        print('Video saved at %s' % filepath)

    def extract_frames(self, file, directory, interval: int = 100, flip: bool = False) -> None:
        """
        Extrernal Code to extract frames from video.
        """
        i = 1
        i_saved = 0
        stream = cv.VideoCapture(file)
        if stream.isOpened():
            for i in tqdm(range(int(stream.get(cv.CAP_PROP_FRAME_COUNT)))):
                ret, frame = stream.read()
                if (i - i_saved) == interval:
                    i_saved = i
                    if flip:
                        frame_saved = cv.flip(cv.flip(frame,0),1)
                    else:
                        frame_saved = frame
                    cv.imwrite(f"{directory[1]}\\frame_{i}.jpg",frame_saved)
                else:
                    pass
                i += 1
        stream.release()
    
    def save_to_numpy(self, array,file_name, file_path):
        """
        Save given numpy array in a given filepath with the given file name
        
        args:
            numpy_array(array): array to be saved into a given file location and given name
            filename(str): file name to be saved as
            file_path(str): file location to be saved to
        """
        
        numpy_array = np.array(array)
        
        with open(os.path.join(os.path.join(self.dir_path,file_path),file_name), 'wb') as f:
            np.save(f,numpy_array)

    def calibrateDistortion(self, camera_name, image_path,  array_path, camWidth, camHeight, imgs = None,):
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
        # _ , camera_name = os.path.split(data_path)
        # array_path = os.path.join(self.dir_path, os.path.join(data_path, 'Arrays'))
        # self.check_for_folder(array_path)


        print(f'Beginning Calibration for {camera_name}'.center(50,'='))
        try:     
            # termination criteria
            criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

            # prepare object points to look for on Chessboard
            cbrow = 5 # Chessboard rows
            cbcol = 4 # Chessboard columns

            print(f'looking for {cbrow} by {cbcol} board')

            objp = np.zeros((cbrow * cbcol, 3), np.float32)
            objp[:, :2] = np.mgrid[0:cbcol, 0:cbrow].T.reshape(-1, 2)

            # Arrays to store object points and image points from all the images.
            objpoints = [] # 3d point in real world space
            imgpoints = [] # 2d points in image plane.

            print(f'Obtaining images form: {image_path}')

            #obtains images in given path
            images = glob.glob(f'{os.path.join(self.dir_path,image_path)}/*.png')


            print(f'\nObtaining Intrinsic values.\n')
            for fname in tqdm(images):

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
                    cv.drawChessboardCorners(calImg, (cbrow,cbcol), corners2, ret)
                    cv.imshow('img', calImg) # Ubuntu has issues displaying images should still run past this step
                    cv.waitKey(1)

                else:
                    # Removes images with no chessboard
                    os.remove(fname)


            cv.destroyAllWindows()

            # # Checks if any callibration data is avalible
            # if ret == False:
            #     raise UnboundLocalError
            
            # Gathers callibration data
            print(f'Obtained Calibration matrix.')
            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            newcameramtx, _ = cv.getOptimalNewCameraMatrix(mtx, dist, (camWidth,camHeight), 1, (camWidth,camHeight))

            
            # Input validation to save data
            print(f'Do you wish so Save and Overide the Calibration Distortion Matrix for {camera_name}')
            if self.conformation('Saving data'):
                
                
                self.save_to_numpy(array=ret,file_name='Ret.npy',file_path= array_path)
                self.save_to_numpy(array=mtx,file_name='Mtx.npy',file_path= array_path)
                self.save_to_numpy(array=dist,file_name='Dist.npy',file_path= array_path)
                self.save_to_numpy(array=rvecs,file_name='Rvecs.npy',file_path= array_path)
                self.save_to_numpy(array=tvecs,file_name='Tvecs.npy',file_path= array_path)
                self.save_to_numpy(array=newcameramtx,file_name='NewCameraMtx.npy',file_path= array_path)
                
                print(f'New Calibration Data has been saved for {camera_name}.')
                




        # ... More Error Handeling ...
        except UnboundLocalError:
            print('No Chess Board Grid found'.center(100,'-'))
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

