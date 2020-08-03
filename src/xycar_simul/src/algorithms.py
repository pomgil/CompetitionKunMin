import numpy as np
import cv2

WIDTH = 640
HEIGHT = 480

pts1 = np.array([[140,290],[500,290],[0,390],[640,390]],np.int32)
pts2 = np.array([[100,0],[540,0],[100,480],[540,480]],np.int32)

def Perspective(image, pts1):
    """
    Capture a Region of Interest
    """
    cv2.line(image,pt1=tuple(pts1[0]),pt2=tuple(pts1[1]),color=(255,0,0),thickness=2)
    cv2.line(image,pt1=tuple(pts1[1]),pt2=tuple(pts1[3]),color=(255,0,0),thickness=2)
    cv2.line(image,pt1=tuple(pts1[3]),pt2=tuple(pts1[2]),color=(255,0,0),thickness=2)
    cv2.line(image,pt1=tuple(pts1[2]),pt2=tuple(pts1[0]),color=(255,0,0),thickness=2)

    cv2.line(image,pt1=tuple(pts2[0]),pt2=tuple(pts2[1]),color=(255,0,0),thickness=2)
    cv2.line(image,pt1=tuple(pts2[1]),pt2=tuple(pts2[3]),color=(255,0,0),thickness=2)
    cv2.line(image,pt1=tuple(pts2[3]),pt2=tuple(pts2[2]),color=(255,0,0),thickness=2)
    cv2.line(image,pt1=tuple(pts2[2]),pt2=tuple(pts2[0]),color=(255,0,0),thickness=2)

    Matrix = cv2.getPerspectiveTransform(pts1.astype(np.float32), pts2.astype(np.float32))
    imgPers = cv2.warpPerspective(image, Matrix, (WIDTH, HEIGHT))
    return imgPers

def Threshold(imgPers):
    imgGray = cv2.cvtColor(imgPers, cv2.COLOR_RGB2GRAY)

    # Blurring -----------------------------------------------
    # imgBlur = cv2.blur(imgPers, (5,5))
    imgBlur = cv2.GaussianBlur(imgGray, (5,5), 0)
    # imgBlur = cv2.medianBlur(imgPers, 5)
    # imgBlur = cv2.bilateralFilter(imgPers,9,75,75)
    # -----------------------------------------------

    imgThresh = cv2.inRange(imgBlur, 149, 255, cv2.THRESH_BINARY)
    cv2.imshow("Binary Threshold", imgThresh)
    cv2.waitKey(1)

    # Erosion / Dilation-------------------------------------------------------
    # kernel = np.ones((5,5),np.uint8)
    # imgErode = cv2.erode(imgThresh,kernel,iterations=1) 
    # imgDilate = cv2.dilate(imgErode,kernel,iterations=1) 
    # -------------------------------------------------------

    # sobel_horizontal = cv2.Sobel(imgErode, cv2.CV_8U, 1, 0, ksize=9)
    # kernel2 = np.ones((3,3),np.uint8)
    # imgMpl = cv2.morphologyEx(sobel_horizontal, cv2.MORPH_CLOSE, kernel2)

    # cv2.imshow("threshold", sobel_horizontal)
    # cv2.waitKey(1)
    
    # Canny edge detection : still need tuning
    imgEdge = cv2.Canny(imgGray, 270, 300)

    cv2.imshow("canny edge", imgEdge)
    cv2.waitKey(1)

    # combing the binary threshold and canny edge detector
    imgFinal = cv2.add(imgThresh, imgEdge)
    imgFinal = cv2.cvtColor(imgFinal, cv2.COLOR_GRAY2RGB)
    imgFinalDuplicate = cv2.cvtColor(imgFinal, cv2.COLOR_RGB2BGR)
    imgFinalDuplicate1 = cv2.cvtColor(imgFinal, cv2.COLOR_RGB2BGR)
    return imgFinal, imgFinalDuplicate, imgFinalDuplicate1

def Histogram(imgFinalDuplicate, imgFinalDuplicate1):
    histogramLane = np.uint8([])
    
    for i in range(WIDTH):
        # ROILane = cv2.rectangle(imgFinalDuplicate,(i,140),(i+1,240),(0,0,255),3)
        # Matrix: rows 100, cols 1 
        ROILane= imgFinalDuplicate[HEIGHT-100:HEIGHT, i]
        # scaling (0 ~ 255) to (0 ~ 1)
        ROILane = cv2.divide(ROILane, 255)
        # summation of white pixels: append the number of white pixels in ROILane to hisogramLane  
        histogramLane = np.append(histogramLane, ROILane.sum(axis=0)[0:1].astype(np.uint8), axis=0)

    return histogramLane


def LaneFinder(imgFinal, histogramLane):
    # find the position of left edge of left lane
    LeftLanePos = np.argmax(histogramLane[:150])

    # find the position of left edge of right lane
    RightLanePos = 590 + np.argmax(histogramLane[540:])

    cv2.line(imgFinal, (LeftLanePos, 0), (LeftLanePos, HEIGHT), color=(0,255,0), thickness=2)
    cv2.line(imgFinal, (RightLanePos, 0), (RightLanePos, HEIGHT), color=(0,255,0), thickness=2)
    return LeftLanePos, RightLanePos

def LaneCenter(imgFinal, LeftLanePos, RightLanePos):
    cv2.putText(imgFinal, str(LeftLanePos), (200,50), 0, 1, color=(0,0,255), thickness=2)
    cv2.putText(imgFinal, str(RightLanePos), (200,100), 0, 1, color=(0,0,255), thickness=2)
    # laneCenter = ((RightLanePos - LeftLanePos) / 2 + LeftLanePos).astype(np.uint8)
    laneCenter = (RightLanePos - LeftLanePos)/2 + LeftLanePos
    cv2.putText(imgFinal, "laneCenter: "+str(laneCenter), (200,150), 0, 1, color=(0,0,255), thickness=2)
    frameCenter = WIDTH // 2

    cv2.line(imgFinal, (laneCenter, 0), (laneCenter, HEIGHT), color=(0,255,0), thickness=3)
    cv2.line(imgFinal, (frameCenter, 0), (frameCenter, HEIGHT), color=(255,0,0), thickness=3)

    Result = frameCenter - laneCenter
    cv2.putText(imgFinal, "Result: "+str(Result), (200,200), 0, 1, color=(0,0,255), thickness=2)
    cv2.imshow("position of white lanes", imgFinal)
    cv2.waitKey(1)
    return Result
