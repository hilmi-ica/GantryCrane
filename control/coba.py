# Import Numpy (math calculation)
import numpy as np

# Import Computer Vision (open and process image or video)
import cv2

# Open Webcam
cap = cv2.VideoCapture(4) # change index camera if needed

# Masking boundaries
# The order of the colors is blue, green, red
# Red-1
lower_color_bounds = (15,15,130)
upper_color_bounds = (85,95,235)
# Red-2
# lower_color_bounds = (10,10,125)
# upper_color_bounds = (90,100,240)
# Purple-1
# lower_color_bounds = (114,90,122)
# upper_color_bounds = (142,109,175)
# Purple-2
# lower_color_bounds = (170,141,174)
# upper_color_bounds = (196,158,202)

# Read previously opened frame
ret,frame = cap.read()
# print(frame)

# Contours for reducing the masking area
contours = np.array([[0,0], [0,150], [640,150], [640,480], [0,480], [0,150], [0,150], [0,480], [640,480],[640,0]])
# contours = np.array([[0,0], [0,150], [500,150], [500,350], [100,350], [100,150], [0,150], [0,480], [640,480],[640,0]])
frame = cv2.fillPoly(frame, pts = [contours], color =(255,255,255))

# Masking
mask = cv2.inRange(frame,lower_color_bounds,upper_color_bounds)
mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
frame = frame & mask_rgb

# Creating kernel
kernel = np.ones((2, 2), np.uint8)

# Using cv2.erode() method 
eroded = cv2.erode(mask, kernel) 

# calculate moments of binary image
M = cv2.moments(eroded)

# calculate x,y coordinate of center
cX_origin = int(M["m10"] / M["m00"])
cY_origin = int(M["m01"] / M["m00"])

# Import ros package (data publishing)
import rospy
from std_msgs.msg import Int16

# Invoke publisher
pub = rospy.Publisher('sway', Int16, queue_size=10)
rospy.init_node('camera', anonymous=True)

# Main function
def talker():
    # Frame loop
    while(True):
        # Read webcam
        ret,capture = cap.read()
        frame = capture
        # print(capture.shape)

        # Contours    
        # contours = np.array([[0,0], [0,150], [450,150], [450,350], [150,350], [150,150], [0,151], [0,480], [640,480],[640,0]])
        frame = cv2.fillPoly(frame, pts = [contours], color =(255,255,255))

        # Masking
        mask = cv2.inRange(frame,lower_color_bounds,upper_color_bounds )
        mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
        frame = frame & mask_rgb

        # Creating kernel
        kernel = np.ones((2, 2), np.uint8)

        # Using cv2.erode() method 
        eroded = cv2.erode(mask, kernel) 
        # print(eroded.shape)

        # calculate moments of binary image
        M = cv2.moments(eroded)

        # calculate x,y coordinate of center
        if M["m00"] == 0:
            cX = cX_origin
            cY = cY_origin
        else:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

        # Locate origin point
        cv2.circle(capture, (cX_origin, cY_origin), 5, (255, 255, 255), -1)
        cv2.putText(capture, "origin", (cX_origin - 25, cY_origin - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Locate sway point
        cv2.circle(capture, (cX, cY), 5, (255, 255, 255), -1)
        cv2.putText(capture, "position", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Show sway line
        cv2.line(capture,(cX_origin, cY_origin),(cX, cY),(0, 0, 255),1)

        # Publish sway data
        rospy.loginfo(cX-cX_origin)  
        pub.publish(int(cX-cX_origin))

        # Show processed frame
        # cv2.imshow('Video',frame)
        cv2.imshow('raw',capture)
        # cv2.imshow('mask',mask)
        cv2.imshow('eroded',eroded)

        # Stop loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            break

# Initialize
if __name__ == '__main__':
    try:
        talker()
    except KeyboardInterrupt:
        pass