 # -*- coding: utf-8 -*-
"""
Created on Sun Nov  8 21:58:01 2020

@author: Tehseen
"""

# This code is used to Find the location of the Origin of the Robotic arm 
# with respect to the image frame. We calculate the center point (origin) of the robotic arm 
# as well as the rotation of the robotic arm with respect to the image frame.
# These values will then be used in the Camera coordinates to the Robotic arm Coordinates Homogenius Transformation

#First of all place the robotic arm base plate on the table below the camera where we will place the robotic arm afterwards
# Then execute the code. The code will detect the Rectangle in the base plate tool then fild the 
# origin and rotation values. 
# At the end we will use these values in our main program.

#[Resources]
# https://stackoverflow.com/questions/34237253/detect-centre-and-angle-of-rectangles-in-an-image-using-opencv
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_begin/py_contours_begin.html#how-to-draw-the-contours
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contour_features/py_contour_features.html#b-rotated-rectangle
# https://stackoverflow.com/questions/52247821/find-width-and-height-of-rotatedrect

import numpy as np
import cv2
import sys
import time
import yaml
import os
import warnings
warnings.filterwarnings("ignore")

#Constants Declaration
webcam_Resolution_Width	= 640.0 #Change the resolution according to your Camera's resolution
webcam_Resolution_Height = 480.0
rectangle_width_in_mm = 49.0    #size of the calibration rectangle (longer side) along x-axis in mm.

# Global Variables
cx = 0.0    #object location in mm
cy = 0.0    #object location in mm
angle = 0.0 #robotic arm rotation angle
one_pixel_length = 0.0  #length of one pixel in cm units
number_of_cm_in_Resolution_width = 0.0  #total number of cm in the camera resolution width


def calculate_XYZ(u,v): #Function to get World Coordinates from Camera Coordinates in mm
        #https://github.com/pacogarcia3/hta0-horizontal-robot-arm/blob/9121082815e3e168e35346efa9c60bd6d9fdcef1/camera_realworldxyz.py#L105        
        cam_mtx =  camera_matrix
        Rt = extrinsic_matrix                    
        #Solve: From Image Pixels, find World Points

        scalingfactor = 40.0 #this is demo value, Calculate the Scaling Factor first (depth)
        tvec1 = Rt[:, 3]  #Extract the 4th Column (Translation Vector) from Extrinsic Matric
        
        uv_1=np.array([[u,v,1]], dtype=np.float32)
        uv_1=uv_1.T
        suv_1=scalingfactor*uv_1
        inverse_cam_mtx = np.linalg.inv(cam_mtx)
        xyz_c=inverse_cam_mtx.dot(suv_1)
        xyz_c=xyz_c-tvec1
        R_mtx = Rt[:,[0,1,2]] #Extract first 3 columns (Rotation Matrix) from Extrinsics Matrix
        inverse_R_mtx = np.linalg.inv(R_mtx)
        XYZ=inverse_R_mtx.dot(xyz_c)
        return XYZ



if __name__ == "__main__":
    while(1):
        try:
            #Start reading camera feed (https://answers.opencv.org/question/227535/solvedassertion-error-in-video-capturing/))
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
   
            #Now Place the base_plate_tool on the surface below the camera.
            while(1):
                _,frame = cap.read()
                # cv2.imshow("Live" , frame)
                k = cv2.waitKey(5)
                if k == 27: #exit by pressing Esc key
                    cv2.destroyAllWindows()
                    sys.exit()
                if k == 13: #Save the centroid and angle values of the rectangle in a file
                    result_file = r'robot_position.yaml'    
                    try:
                        os.remove(result_file)  #Delete old file first
                    except:
                        pass
                    print("Saving Robot Position Matrices .. in ",result_file)
                    cx = (cx * one_pixel_length)/10.0 #pixel to cm conversion
                    cy = (cy * one_pixel_length)/10.0
                    data={"robot_position": [cx,cy,angle,number_of_cm_in_Resolution_width]}
                    with open(result_file, "w") as f:
                        yaml.dump(data, f, default_flow_style=False)
                
                
                red = np.matrix(frame[:,:,2])  #extracting red layer (layer No 2) from RGB
                green = np.matrix(frame[:,:,1]) #extracting green layer (layer No 1) from RGB
                blue = np.matrix(frame[:,:,0])  #extracting blue layer (layer No 0) from RGB
                #it will display only the Blue colored objects bright with black background
                blue_only = np.int16(blue)-np.int16(red)-np.int16(green)
                blue_only[blue_only<0] =0
                blue_only[blue_only>255] =255
                blue_only = np.uint8(blue_only)            
                # cv2.namedWindow('blue_only', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow("blue_only",blue_only)
                # cv2.waitKey(1)
                
                #https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_thresholding/py_thresholding.html#otsus-binarization
                #Gaussian filtering
                blur = cv2.GaussianBlur(blue_only,(5,5),cv2.BORDER_DEFAULT)
                #Otsu's thresholding
                ret3,thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
                cv2.namedWindow('Threshold', cv2.WINDOW_AUTOSIZE)
                cv2.imshow("Threshold",thresh)
                cv2.waitKey(1)
                contours,hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    
    
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area>100000:
                        contours.remove(contour)
                 
                cnt = contours[0] #Conture of our rectangle
                
                ##############################################################
                #https://stackoverflow.com/a/34285205/3661547
                #fit bounding rectangle around contour            
                rotatedRect = cv2.minAreaRect(cnt)
                #getting centroid, width, height and angle of the rectangle conture
                (cx, cy), (width, height), angle = rotatedRect
                
                
                #centetoid of the rectangle conture
                cx=int(cx)
                cy=int(cy)
                # print (cx,cy) #centroid of conture of rectangle
                
                #Location of Rectangle from origin of image frame in millimeters
                x,y,z = calculate_XYZ(cx,cy)
                               
                #but we choose the Shorter edge of the rotated rect to compute the angle between Vertical
                #https://stackoverflow.com/a/21427814/3661547
                if(width > height):
                    angle = angle+180
                else:
                    angle = angle+90
                # print("Angle b/w shorter side with Image Vertical: \n", angle)
                
                #cm-per-pixel calculation
                if(width != 0.0):
                    one_pixel_length = rectangle_width_in_mm/width #length of one pixel in mm (rectangle_width_in_mm/rectangle_width_in_pixels)
                    number_of_cm_in_Resolution_width = (one_pixel_length*640)/10 #in cm #Change the resolution according to your Camera's resolution
                    print(number_of_cm_in_Resolution_width)
                
                
                ##############################################################
                
                
                #Draw rectangle around the detected object
                #https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_contours/py_contours_begin/py_contours_begin.html#how-to-draw-the-contours
                im = cv2.drawContours(frame,[cnt],0,(0,0,255),2)
                # cv2.namedWindow('Contours', cv2.WINDOW_AUTOSIZE)
                # cv2.imshow("Contours",im)
                # cv2.waitKey(1)
                
                cv2.circle(im, (cx,cy), 2,(200, 255, 0),2) #draw center
                cv2.putText(im, str("Angle: "+str(int(angle))), (int(cx)-40, int(cy)+60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
                cv2.putText(im, str("Center: "+str(cx)+","+str(cy)), (int(cx)-40, int(cy)-50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1, cv2.LINE_AA)
                cv2.namedWindow('Detected Rect', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('Detected Rect',im)
                cv2.waitKey(1)

                
                
        except Exception as e:
            print("Error in Main Loop\n",e)
            cv2.destroyAllWindows()
            sys.exit()
    
    cv2.destroyAllWindows()


