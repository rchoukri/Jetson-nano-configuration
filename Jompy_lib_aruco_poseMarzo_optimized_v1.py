# Importing Librairies 
from turtle import pos
import cv2    #4.5.4.58
from cv2 import putText

import numpy as np   #1.22.4
import cv2.aruco as aruco #4.5.4.58
import sys, time, math
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
                camera_size=[640,480]  #dimensions of camera
                ):

        self.id_to_find     = id_to_find
        self.marker_size    = marker_size
        self._show_video    = show_video
        self.camera_name    = camera_name
        self._camera_matrix = camera_matrix
        self._camera_distortion = camera_distortion
        self.poseCamara=poseCamara
        self.angulos_cam=angulos_cam
        print(self._camera_matrix)
        print(self._camera_distortion)
        self.is_detected    = False
        self._kill          = False

        #--- Define the aruco dictionary
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self._parameters  = aruco.DetectorParameters_create()

        #-- Font for the text in the image
        self.font = cv2.FONT_HERSHEY_PLAIN
        self._t_read      = time.time()
        self._t_detect    = self._t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0
    
    # Remember that the rotations are made relative to the right hand ruler.
        # relative to the axis of rotation (to determine its direction)
        # alpha in rad
        
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
    
    def camera_to_uav_v2(self,poseCamara,rotaciones):
        """ Hay que tener en cuenta la orientación del zócalo de camara y el 
            posicionamiento del sistema de la cámara con el zócalo donde los éjes de la 
            cámara corresponden a la siguiente relación con el zócalo x_cam=y_zocalo
            y_cam=z_zucalo, z_cam=x_zocalo, esto es equicvalente a rotar [90,0,90] """

        # For the composition of transformations we add to the left of the last transformation the following operation
        # Ex: A then B then C would be C*B*A leaving on the right side the first transformation and on the left side the last one.
        
        #Transformation between the camera and the gimbal socket 
        R_cz=self.rotZ(90)*self.rotY(0)*self.rotX(90) #[Yaw, Pithc, Roll] # Rotación cámara zócalo
        T_cz=self.traslacion([0,0,0])
        TT_cz=T_cz*R_cz

        # Transformation between the camera and the body (UAV)
        R_zb=self.rotZ(rotaciones[2])*self.rotY(rotaciones[1])*self.rotX(rotaciones[0])
        T_zb=self.traslacion(poseCamara)
        TT_zb=T_zb*R_zb # Transformación zócalo body rototraslación 
        
        #Transformation between the camera and the body (UAV)
        TT_cb=TT_zb*TT_cz
        R_cb=R_zb*R_cz
        TT_bc=np.linalg.inv(TT_cb)
        return(R_cb,TT_cb, TT_bc)

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
        
    def correccion_cilindrica(Self,x,y,z,theta,r,d):
        #We apply a rotation followed by a translation by radial bias.
        c=math.cos(math.radians(theta))
        s=math.sin(math.radians(theta))

        #angular correction  (rotation)
        X=x*c-y*s
        Y=x*s+y*c

        #Radial correction on the polar projection
        rr=np.linalg.norm([X,Y])-r
        theta_r=np.arctan2(Y,X)

        #Transformed back to Cartesian
        X=rr*math.cos(theta_r)
        Y=rr*math.sin(theta_r)
        Z=z-d

        return (X,Y,Z)
    def track(self, loop=True, verbose=False, show_video=None):

        self._kill = False
        if show_video is None: show_video = self._show_video
        marker_found = False
        x = y = z = 0
        yaw_UAV=0

        while not self._kill:

            #-- Read the camera frame
            self._cap = cv2.VideoCapture(0)
    
            ret, frame = self._cap.read() #returns ret and the frame
            font,fontFace,fontScale,thickness,textOrg = self.font_for_text()
            if cv2.waitKey(1) & 0xFF == ord('q'): #will display a frame for 1 ms
                break

            self._update_fps_read()

            frame=cv2.putText(frame,('FPS %.0f' %self.fps_read),textOrg, fontFace, fontScale,(255,0,255),thickness)

            #-- Convert in gray scale
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
           
            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self._aruco_dict,
                            parameters=self._parameters
                             )   
                                 
            if not ids is  None and self.id_to_find in ids[0]:  
                marker_found = True
                self._update_fps_detect()
               
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self._camera_matrix, self._camera_distortion)

                #-- Unpack the output, get only the first
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                centro=np.matrix([0,0,0,1]).T #Coordinates of  center

                #correction
                tvec2=tvec+[0.0,0.0,-0.15]
                #tvec2=tvec
               
                x = tvec2[0]
                y = tvec2[1]
                z = tvec2[2]
                
                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners) #DA ERROR DENTRO DE AIRSIM
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

                # Como la cámara puede estar colocada en otra posición que no sea el centro del UAV

                
                x = Aruco_uav_ref[0]  
                y = Aruco_uav_ref[1]
                z = Aruco_uav_ref[2]
               
               
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
    id_to_find  = 20
    marker_size  = 0.165 #- [m] #The relationship is given by the calibration matrices

    #--- Get the camera calibration path
    calib_path  =  "C:/Users/chouk/OneDrive/Bureau/Nouveau dossier/Scripts/calibration/"
    

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

    poseCamara=[0,0,0.1] #[X,Y,Z]
    angulos_cam=[0,-90,0] #[Roll, Pitch, Yaw]
    aruco_tracker = ArucoSingleTracker(id_to_find, marker_size, camera_matrix=camera_matrix,
                                         camera_distortion=camera_distortion,camera_name=camera_name,
                                         show_video=True, poseCamara=poseCamara,angulos_cam=angulos_cam)
    

    aruco_tracker.track(verbose=True)
    
