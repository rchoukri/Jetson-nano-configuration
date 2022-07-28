#This Library has been debuged and commented on 20/06/2022
#Change the calib_path on the line 425 (instruction in lines 417 to 423)

#Packages versions
'''To reuse this library you have to pay attention to the :
versions of the packages used: The latest version of OpenCV is not adapted to this code because there are some functions used in this program from the cv2.aruco library, which does not work, to solve this problem it is necessary to install the version 4.5.4.58 of Opencv.
NB: the 'opencv-python' library does not work (the name Aruco is not recognized), it is necessary to use opencv-contrib-python and uninstall opencv-python, for that it is enough to type the following commands:
- pip uninstall opencv-python
- pip uninstall opencv-contrib-python
-pip install opencv-contrib-python==4.5.4.58

for the other packages installed we didn't encounter any problems with the current versions which are:
Numpy: 1.22.4
Yaml: 6.0
'''
#Introduction
'''
This library aims to detect a specific Aruco code, the aruco codes are defined by their ID [0-999] and their dimensions (4x4,5x5,6x6 or 7x7),
thereafter we will work on an Aruco code of ID 20 and dimension 5X5.
The specificity of this library is that it is adapted to a use with the UAV,
it allows the detection of the helipad (Aruco code) for the landing, for that we use a set of functions that allow the detection of the Aruco code (borders, corners, ids), 
and other functions to display the detected parameters on the frame,
and finally other functions to adapt the detected parameters with the position of the drone to have to have the desired accuracy for the landing to go through without any problem,
to better understand this last type of functions, it is advisable to take a look at the article:
Error Reduction in Vision-Based Multirotor Landing System (sensors): https://www.mdpi.com/1424-8220/22/10/3625
'''



#reference frames:
'''
TARGET:
                A y
                |
                |
                |tag center
                O---------> x
CAMERA:
                O--------> x
                | frame center
                |
                |
                V y
SOCKET:

                O---------->y
                |
                |
                |
                V z
UAV:                      
                O---------->y
                |
                |
                |
                z    


F1: Flipped (180 deg) tag frame around x axis
F2: Flipped (180 deg) camera frame around x axis
The attitude of a generic frame 2 respect to a frame 1 can obtained by calculating euler(R_21.T)
We are going to obtain the following quantities:
    > from aruco library we obtain tvec and Rct, position of the tag in camera frame and attitude of the tag (lines 303-305)
    > position of the Camera in Tag axis: R_ct.T*tvec (line 323)
    > Transformation of the camera, respect to f1 (the tag flipped frame): R_cf1 = R_ct*R_tf1 = R_cf*R_f
    > Transformation of the tag, respect to f2 (the camera flipped frame): R_tf2 = Rtc*R_cf2 = R_tc*R_f
    > R_tf1 = R_cf2 an symmetric = R_f

'''

#In the following we will detail each of the functions used.






# Importing Librairies 
import sys, time, math #pre-installed packages

import cv2    #4.5.4.58
from cv2 import putText
import cv2.aruco as aruco 

import numpy as np   #1.22.4
import yaml  #6.0


#Class definition
class ArucoSingleTracker():
    def __init__(self,
                id_to_find,
                marker_size,
                camera_matrix,
                camera_distortion,
                camera_name,
                show_video,
                poseCamara,
                angulos_cam,
                camera_size=[640,480]  #resolution of camera
                ):
        #initialization of parameters
        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._show_video    = show_video
        self.camera_name    = camera_name
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        self.poseCamara=poseCamara
        self.angulos_cam=angulos_cam
        self.is_detected    = False
        self._kill          = False

        #--- Define the aruco dictionary
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250) #If dimensions of aruco are different, you must change the dictionnary, here we use a 5X5 Aruco
        self._parameters  = aruco.DetectorParameters_create()
        
        

        #-- Font for the text in the image
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0
    
    # Remember that the rotations are made relative to the right hand ruler.
    # relative to the axis of rotation (to determine its direction)
    # alpha in rad


    """It is necessary to take into account the orientation of the camera 
        base and the positioning of the camera system with the base where the 
        camera axes correspond to the following relation with the base 
        x_cam=y_base y_cam=z_z_socket, z_cam=x_socket, this is equivalent to rotate [90,0,90] (Line 188).
        
        For this we will define four functions rotX, rotY, rotZ and traslacion, which will allow us to calculate the homogeneous transformations
         (Cf Article:  Error Reduction in Vision-Based Multirotor Landing System (sensors)) """

        
     #X Axis Rotation   
    def rotX(self,alpha):
        
        alpha=np.deg2rad(alpha)
        rot_x = np.array([[1, 0, 0],
                        [0, np.cos(alpha), -np.sin(alpha)],
                        [0, np.sin(alpha), np.cos(alpha)]])

        rot_x=np.vstack((np.hstack((rot_x, np.matrix([0, 0, 0]).T)),[0, 0, 0 ,1]))
        return (rot_x)

    #Y Axis Rotation
    def rotY(self,alpha):
        alpha=np.deg2rad(alpha)
        rot_y = np.array([[np.cos(alpha), 0, np.sin(alpha)],
                        [0, 1, 0],
                        [-np.sin(alpha), 0, np.cos(alpha)]])
        rot_y=np.vstack((np.hstack((rot_y, np.matrix([0, 0, 0]).T)),[0, 0, 0 ,1]))
        return (rot_y)

    #Z Axis Rot
    def rotZ(self,alpha):
        alpha=np.deg2rad(alpha)
        rot_z = np.array([[np.cos(alpha), -np.sin(alpha), 0],
                        [np.sin(alpha), np.cos(alpha), 0],
                        [0, 0, 1]])
        rot_z=np.vstack((np.hstack((rot_z, np.matrix([0, 0, 0]).T)),[0, 0, 0 ,1]))
        return (rot_z)

    # Homogeneous translation matrix
    def traslacion(self,p):
        I = np.identity(3)
        p_homo=np.vstack((np.hstack((I, np.matrix(p).T)),[0, 0, 0 ,1]))
        return (p_homo)
     
    """ The function  camera_to_uav_v2() allow us to calculate:
    R_cb: Rotation respect Camera and body,
    TT_cb: Transformation respect camera and body,
    TT_bc: Transformation respect Body and camera """

    #the matematic model for this transformations is in the article: Error Reduction in Vision-Based Multirotor Landing System 
    def camera_to_uav_v2(self,poseCamara,rotaciones):
      
        # For the composition of transformations we add to the left of the last transformation the following operation
        # Ex: A then B then C would be C*B*A leaving on the right side the first transformation and on the left side the last one.
        
        #Transformation respect the camera and the gimbal socket 
        R_cz=self.rotZ(90)*self.rotY(0)*self.rotX(90) #[Yaw, Pithc, Roll] # rotation respect camera and socket
        T_cz=self.traslacion([0,0,0])
        TT_cz=T_cz*R_cz

        # Transformation respect the camera and the body (UAV)
        R_zb=self.rotZ(rotaciones[2])*self.rotY(rotaciones[1])*self.rotX(rotaciones[0]) #rotaciones is a vector who containes [0,-90,0] and it represents the angle beetween the Camera and the Body
        T_zb=self.traslacion(poseCamara)
        TT_zb=T_zb*R_zb
        
        #Transformation respect the camera and the body (UAV)
        TT_cb=TT_zb*TT_cz
        R_cb=R_zb*R_cz
        TT_bc=np.linalg.inv(TT_cb)
        return(R_cb,TT_cb, TT_bc)


    """in order to calculate the angles roll_camera, pitch_camera, yaw_camera, we use the function rotationMatrixToEulerAngles(),
     it takes as argument a rotation matrix, so we have to make sure that the parameter is a rotation matrix,
      for that we define the boolean function isRotationMatrix() which allows to know if the matrix is a rotation matrix"""
     # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def _rotationMatrixToEulerAngles(self,R):
        # Calculates rotation matrix to euler angles
        # The result is the same as MATLAB except the order
        # of the euler angles ( x and z are swapped ).
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    '''in this part (line 239 to line 259 )we define the outputs that the users will see on the screen, 
    we define the fps(Frame per second) using the python time library, 
    we also define in the font_for_text() function the size, the font format and the writing location'''

    def _update_fps_read(self):
        t           = time.time()
        self.fps_read    = 1.0/(t - self._t_read)
        self._t_read      = t

    def _update_fps_detect(self):
        t           = time.time()
        self.fps_detect  = 1.0/(t - self._t_detect)
        self._t_detect      = t

    def stop(self):
        self._kill = True

    def font_for_text(self):
        font = cv2.FONT_HERSHEY_PLAIN
        fontFace = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 0.5
        thickness = 2
        textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)
        textOrg = (10, 10 + textSize[1])
        return(font,fontFace,fontScale,thickness,textOrg)

    #definition of the tracking function 
    def track(self, loop=True, verbose=False, show_video=None):
  
        self._kill = False
        if show_video is None: show_video = self._show_video
        marker_found = False
        x = y = z = 0
        yaw_UAV=0
        self._cap = cv2.VideoCapture(0) #video capture (frame)
        while not self._kill:

            
            #-- Read the camera frame
            
            ret, frame = self._cap.read() #returns ret and the frame
            
            font,fontFace,fontScale,thickness,textOrg = self.font_for_text() #initialistion of parametres to write text on the frame
            
            
  
            if cv2.waitKey(1) & 0xFF == ord('q'): #will display a frame for 1 ms and stop by pressing 'q'
                break

            self._update_fps_read() #calculate the new fps after reading the next frame
            
            frame=cv2.putText(frame,('FPS %.0f' %self.fps_read),textOrg, fontFace, fontScale,(255,0,255),thickness)  #write the parametrs  on the frame
            
            #-- Convert in gray scale
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
           
            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict,
                            parameters=self._parameters
                             )   
                                 
            if not ids is  None and self.id_to_find in ids[0]:  #we are looking for a specific Aruco, defined by its id.
                marker_found = True
                self._update_fps_detect()
                """The function estimaPoseSingleMarker receives in parameters the detected marker or more precisely the corners of the marker,its size, and the parameters of the camera.
                And returns the camera pose with respect to the marker and that is the transformation that transforms points from the marker’s coordinate
                system to the camera’s coordinate system. The camera pose with respect to a marker is the 3d transformation from the marker coordinate system
                 to the camera coordinate system. It is specified by rotation (rvecs) and translation (tvecs) vectors. """
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)
                #-- Unpack the output, get only the first
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:] #extract the rotation and translation vector from ret ret
                
                centro=np.matrix([0,0,0,1]).T #Coordinates of  center

                #correction of bias, to change every time we change the camera settings
                tvec2=tvec+[0.0,0.0,-0.15] #actually we use a manual correction, maybe we can automate this correction 
                
                #tvec2=[x,y,z]
                x = tvec2[0]
                y = tvec2[1]
                z = tvec2[2]
                
                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners) 
                aruco.drawAxis(frame, self._camera_matrix, self._camera_distortion, rvec, tvec, self.marker_size/2) 

                #------- Obtain the transformation matrix target->camera
                R_ct= np.matrix(cv2.Rodrigues(rvec)[0]) # Rotation matrix marker respect to camera frame
                TT_ct=np.vstack((np.hstack((R_ct,np.matrix(tvec2).T)),[0, 0, 0 ,1]))

                R_tc= np.linalg.inv(R_ct) # Rotation matrix camera respect to marker  
                TT_tc=np.vstack((np.hstack((R_tc,np.matrix(tvec2).T)),[0, 0, 0 ,1]))
            


                roll_marker, pitch_marker, yaw_marker = self._rotationMatrixToEulerAngles(R_ct)

                #-- Now get Position and attitude of the camera respect to the marker
                pos_camera = TT_tc*centro
                roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(R_tc)
                
                #-- Drone body camera transformation
                R_cb,TT_cb,TT_bc=self.camera_to_uav_v2(self.poseCamara,self.angulos_cam)
                TT_tb=TT_cb*TT_tc
                tvec_uav=TT_tb*centro
                Aruco_uav_ref=tvec_uav[:3,:] # We dispense with the final term of the homogeneous vector 
                roll_UAV, pitch_UAV, yaw_UAV = self._rotationMatrixToEulerAngles(TT_tb[:3,:3])

               #As the camera can be placed in a position other than the center of the UAV

                x = Aruco_uav_ref[0]  
                y = Aruco_uav_ref[1]
                z = Aruco_uav_ref[2]
                #x,y and z are  matrices of a single element ([x,y,z]=[matrix([[-0.04662191]]), matrix([[0.06815405]]), matrix([[0.40045108]])])

                #we get this element :
                x=x[0,0]
                y=y[0,0]
                z=z[0,0]
                
               
                if show_video: 
                    if False:
                        #-- Print the tag position and attitude in camera frame (respect to camera frame)
                        str_position = "MARKER Position x=%4.2f  y=%4.2f  z=%4.2f"%(tvec2[0], tvec2[1], tvec2[2])
                        cv2.putText(frame, str_position, (0, 50), self.font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    
                        #-- Print the marker's attitude respect to camera frame
                        str_attitude = "MARKER Attitude Roll=%4.2f  Pitch=%4.2f  Yaw=%4.2f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                        cv2.putText(frame, str_attitude, (0, 75), font, 1, (0, 255,0), 2, cv2.LINE_AA)
                        #---------------------------------------------------------------
                        #------------------------- Camera frame reference-----------------
                        str_position = "CAMERA Position x=%4.2f  y=%4.2f  z=%4.2f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                        cv2.putText(frame, str_position, (0, 100), self.font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        #-- Get the attitude of the camera respect to the frame
                        #roll_camera, pitch_camera, yaw_camera = self._rotationMatrixToEulerAngles(self._R_flip*R_tc)
                        str_attitude = "CAMERA Attitude Roll=%4.2f  Pitch=%4.2f  Yaw=%4.2f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                            math.degrees(yaw_camera))
                        cv2.putText(frame, str_attitude, (0, 125), self.font, 1, (0, 0, 255), 2, cv2.LINE_AA)
                        #--------------------------------------------------------
                        #---------------------------- body frame reference----------------------------
                        
                        str_attitude = "UAV Attitude Roll=%4.2f  Pitch=%4.2f  Yaw=%4.2f"%(
                                                                math.degrees(roll_UAV),math.degrees(pitch_UAV),
                                                                math.degrees(yaw_UAV))
                        str_position = "UAV Position x=%4.2f  y=%4.2f  z=%4.2f"%(x,y,z)
                        cv2.putText(frame, str_position, (0, 150), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        cv2.putText(frame, str_attitude, (0, 175), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        #------------------------------------------------------------------------------
                    else:
                        #---------------------------- body frame reference----------------------------
                        
                        str_attitude = "Attitude: Roll=%4.2f  Pitch=%4.2f  Yaw=%4.2f"%(
                                                                math.degrees(roll_UAV),math.degrees(pitch_UAV),
                                                                math.degrees(yaw_UAV))
                        str_position = "Position: x=%4.2f  y=%4.2f  z=%4.2f"%(x,y,z)
                        cv2,putText(frame,"Target from UAV position:",(0, 50), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        cv2.putText(frame, str_position, (0, 75), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
                        cv2.putText(frame, str_attitude, (0, 100), font, 1, (255, 0, 0), 2, cv2.LINE_AA)
            else:
                if verbose: print ("Nothing detected - fps = %.0f"%self.fps_read)


            if show_video:
                #--- Display the frame
                cv2.imshow(self.camera_name, frame)
                #--- use 'q' to quit
                key = cv2.waitKey(1) & 0xFF 
                if key == ord('q'):
                    self._cap.release()
                    cv2.destroyAllWindows()
                    break

            if not loop: return(marker_found, x, y, z,math.degrees(yaw_UAV))

if __name__ == "__main__":
    
    #--- Define Tag
    id_to_find  = 20  #In our case we look for an Aruco with ID=20 and size=5X5
    marker_size  = 0.165 #- [m] 

    #--- Get the camera calibration path
    #In calibration folder, we must have 'CameraMatrix.txt and cameraDistorsion.txt'

    '''To get the two files CameraMatrix.txt and cameraDistorsion.txt we use two other programs:
The first one is Manual_DataGeneration.py: we take several pictures of a chessboard (minimuim 12) in order to have a set of chessboard images, once we finish taking the pictures,
we use the second program Camera_calib.py to calibrate the Camera, and generate the two files CameraMatrix.txt and cameraDistorsion.txt, which contain the parameters of the used camera, 
, this procedure must be done for each new camera used'''

    calib_path  =  "C:/Users/chouk/OneDrive/Bureau/stage/Nouveau dossier/Scripts/calibration/"
    
    if False:
        path=calib_path+'calibresult.png'
        with open(path) as f:
            loadeddict = yaml.load(f,Loader=yaml.FullLoader)
        camera_matrix = loadeddict.get('camera_matrix')
        camera_distortion = loadeddict.get('dist_coeff')
        camera_matrix = np.array(camera_matrix)
        camera_distortion = np.array(camera_distortion)
    else:
        camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
        camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')

    camera_name = 'pruebas'

    #The angle between the body and the socket is 90° along the Y axis and taking into account the offset between
    #  the position of the body and the Camera which is 0.1m along the z axis,
    #  we will introduce the two variables poseCamara and angulos_cam to correct this shift

    poseCamara=[0,0,0.1] #[X,Y,Z] 
    angulos_cam=[0,-90,0] #[Roll, Pitch, Yaw]
    
    aruco_tracker = ArucoSingleTracker(id_to_find, marker_size, camera_matrix=camera_matrix,
                                         camera_distortion=camera_distortion,camera_name=camera_name,
                                         show_video=True, poseCamara=poseCamara,angulos_cam=angulos_cam)
    aruco_tracker.track(verbose=True)
 


    

