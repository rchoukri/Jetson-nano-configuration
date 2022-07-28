##: ESTE SCRIPT SIRVE PARA OBTENER LOS VALORES NECESARIOS DE CALIBRACIÓN DE LA CÁMARA UNA VEZ HAN SIDO TOMADAS LAS IMÁGENES
##: ---------------------------------------------------------------
##: THIS SCRIPT IS USED FOR CAMERA CALIBRATION ONCE PICTURES HAVE BEEN TAKEN

"""
From https://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html#calibration
Calling:
cameracalib.py  <folder> <image type> <num rows> <num cols> <cell dimension>
like cameracalib.py folder_name png
--h for help
"""


import numpy as np
import cv2
import glob
import sys
import argparse
import yaml





#---------------------- SET THE PARAMETERS

""" Hay que recordar que buscamos las esquinas por lo que un tablero de 
    filas X columnas=9x6 cuadrados va tener 8x5 esqunas """

nRows = 7   #8
nCols = 5
dimension = 200 #- mm

workingFolder   = "chess_image_calibration_flight/"
imageType       = 'jpg'
#------------------------------------------

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, dimension, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((nRows*nCols,3), np.float32)
objp[:,:2] = np.mgrid[0:nCols,0:nRows].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

if len(sys.argv) < 6:
        print("\n Not enough inputs are provided. Using the default values.\n\n" \
              " type -h for help")
else:
    workingFolder   = sys.argv[1]
    imageType       = sys.argv[2]
    nRows           = int(sys.argv[3])
    nCols           = int(sys.argv[4])
    dimension       = float(sys.argv[5])

if '-h' in sys.argv or '--h' in sys.argv:
    print("\n IMAGE CALIBRATION GIVEN A SET OF IMAGES")
    print(" call: python cameracalib.py <folder> <image type> <num rows (9)> <num cols (6)> <cell dimension (25)>")
    print("\n The script will look for every image in the provided folder and will show the pattern found." \
          " User can skip the image pressing ESC or accepting the image with RETURN. " \
          " At the end the end the following files are created:" \
          "  - cameraDistortion.txt" \
          "  - cameraMatrix.txt \n\n")

    sys.exit()

# Find the images files
filename    = workingFolder + "/*." + imageType
images      = glob.glob(filename)

print(len(images))
if len(images) < 9:
    print("Not enough images were found: at least 9 shall be provided!!!")
    sys.exit()

else:
    nPatternFound = 0
    imgNotGood = images[1]

    for fname in images:
        if 'calibresult' in fname: continue
        #-- Read the file and convert in greyscale
        img     = cv2.imread(fname)
        gray    = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        print("Reading image ", fname)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (nCols,nRows),None)
        print(ret)
        # If found, add object points, image points (after refining them)
        if ret == True:
            print("Pattern found! Press ESC to skip or ENTER to accept")
            #--- Sometimes, Harris cornes fails with crappy pictures, so
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (nCols,nRows), corners2,ret)
            cv2.imshow('img',img)
            # cv2.waitKey(0)
            k = cv2.waitKey(0) & 0xFF
            if k == 27: #-- ESC Button
                print("Image Skipped")
                imgNotGood = fname
                continue

            print("Image accepted")
            nPatternFound += 1
            objpoints.append(objp)
            imgpoints.append(corners2)
            print(nPatternFound)

            # cv2.waitKey(0)
        else:
            imgNotGood = fname


cv2.destroyAllWindows()

if (nPatternFound > 1):
    print("Found %d good images" % (nPatternFound))
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    # Undistort an image
    img = cv2.imread(imgNotGood)
    h,  w = img.shape[:2]
    print("Image to undistort: ", imgNotGood)
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # undistort
    mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
    dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]
    print("ROI: ", x, y, w, h)

    cv2.imwrite(workingFolder + "/calibresult.png",dst)
    print("Calibrated picture saved as calibresult.png")
    print("Calibration Matrix: ")
    print(mtx)
    print("Disortion: ", dist)

    #--------- Save result------------------------------
    filename = workingFolder + "/cameraMatrix.txt"
    np.savetxt(filename, mtx, delimiter=',')
    filename = workingFolder + "/cameraDistortion.txt"
    np.savetxt(filename, dist, delimiter=',')

    #----------------Guardamos en otro formato------------
    data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}
    with open("calibrationAirsim_v3.yaml", "w") as f:
        yaml.dump(data, f)

    mean_error = 0 # creo que es el mean absolute error 
    #mean_error = np.zeros((1, 2))
    #square_error = np.zeros((1, 2))
    mean_error2 = 0
    square_error = 0
    n_images = len(objpoints)

    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error

        # Jompy aportación
        """errorJP = imgpoints2.reshape(-1, 2) - imgpoints[i]
        mean_error2 += abs(errorJP).sum(axis=0) / len(imgpoints[i])
        square_error += (errorJP**2).sum(axis=0) / len(imgpoints[i])"""
    
    #square_error = np.sqrt(square_error.sum()/ n_images)
    #mean_error2 = cv2.norm(mean_error2 / n_images)

    print("total error: ", mean_error/len(objpoints))
    #print("mean Error: ",mean_error2)
    #print("RMS ",square_error )

else:
    print("In order to calibrate you need at least 9 good pictures... try again")

# Aportaci´n Jompy
def reprojection_error_ext(objp, imgp, cameraMatrix, distCoeffs, rvecs, tvecs):
    """
    Returns the mean absolute error, and the RMS error of the reprojection
    of 3D points "objp" on the images from a camera
    with intrinsics ("cameraMatrix", "distCoeffs") and poses ("rvecs", "tvecs").
    The original 2D points should be given by "imgp".
    
    See OpenCV's doc about the format of "cameraMatrix", "distCoeffs", "rvec" and "tvec".
    """
    
    mean_error = np.zeros((1, 2))
    square_error = np.zeros((1, 2))
    n_images = len(imgp)

    for i in range(n_images):
        imgp_reproj, jacob = cv2.projectPoints(
                objp[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs )
        error = imgp_reproj.reshape(-1, 2) - imgp[i]
        mean_error += abs(error).sum(axis=0) / len(imgp[i])
        square_error += (error**2).sum(axis=0) / len(imgp[i])

    mean_error = cv2.norm(mean_error / n_images)
    square_error = np.sqrt(square_error.sum() / n_images)
    
    return mean_error, square_error