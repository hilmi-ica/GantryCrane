# PATH Setup (Change CHECKPOINT_PATH and CUSTOM_MODEL_NAME)
ANNOTATION_PATH = "annotations"
MODEL_PATH = "models"
CHECKPOINT_PATH = MODEL_PATH+'/my_ssd_mobnet/'
CUSTOM_MODEL_NAME = "my_ssd_mobnet"
CONFIG_PATH = MODEL_PATH + '/' + CUSTOM_MODEL_NAME + '/pipeline.config'

# Import Operating System (open file)
import os
from platform import release

# Import Computer Vision (webcam purposes)
import cv2

# Import time (fps)
import time

# Import mmap (image exchange despite ROS)
import mmap

# Import Numpy (math calculation)
import numpy as np

# Import Tensorflow Module and API
import tensorflow as tf
from object_detection.utils import config_util
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
from object_detection.builders import model_builder

# Import ROS Packages
import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import Int16

# Invoke publisher
pub1 = rospy.Publisher('object_point', UInt16, queue_size=10)
pub2 = rospy.Publisher('sway', Int16, queue_size=10)
rospy.init_node('camera_node', anonymous=False)

# Load pipeline config and build a detection model
configs = config_util.get_configs_from_pipeline_file(CONFIG_PATH)
detection_model = model_builder.build(model_config=configs['model'], is_training=False)

# Restore checkpoint
ckpt = tf.compat.v2.train.Checkpoint(model=detection_model)
ckpt.restore(os.path.join(CHECKPOINT_PATH, 'ckpt-6')).expect_partial()

# Detection function
@tf.function
def detect_fn(image):
    image, shapes = detection_model.preprocess(image)
    prediction_dict = detection_model.predict(image, shapes)
    detections = detection_model.postprocess(prediction_dict, shapes)
    return detections

# Load category index
category_index = label_map_util.create_category_index_from_labelmap(ANNOTATION_PATH+'/label_map.pbtxt')

# Setup camera capture
# cap1 = cv2.VideoCapture(2) # Camera dynamic(1) # Change camera index if needed
cap1 = cv2.VideoCapture(4) 
cap1.release() # Prevent previous unreleased session

# cap = cv2.VideoCapture(4) # Camera static(2) # Change camera index if needed
# cap.release() # Prevent previous unreleased session

# cap1 = cv2.VideoCapture(2) # Camera dynamic(1) # Change camera index if needed
cap1 = cv2.VideoCapture(4)
width1 = int(cap1.get(cv2.CAP_PROP_FRAME_WIDTH))
height1 = int(cap1.get(cv2.CAP_PROP_FRAME_HEIGHT))

# cap = cv2.VideoCapture(4) # Camera static(2) # Change camera index if needed
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Masking boundaries
# The order of the colors is blue, green, red
# Red-1
# lower_color_bounds = (20,20,135)
# upper_color_bounds = (80,90,230)
# Red-2
# lower_color_bounds = (10,10,125)
# upper_color_bounds = (90,100,240)
# Purple-1
# lower_color_bounds = (114,90,122)
# upper_color_bounds = (142,109,175)
# Purple-2
lower_color_bounds = (146,117,117)
upper_color_bounds = (164,117,135)

# Read frame to determine origin
ret1,frame1 = cap1.read()
# print(frame)

# Contour to limit the workload
contours = np.array([[0,0], [0,150], [500,150], [500,480], [100,480], [100,150], [0,150], [0,480], [640,480],[640,0]])
frame1 = cv2.fillPoly(frame1, pts = [contours], color =(255,255,255))

# Masking
mask = cv2.inRange(frame1,lower_color_bounds,upper_color_bounds)
mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
frame1 = frame1 & mask_rgb

# Creating kernel
kernel = np.ones((2, 2), np.uint8)

# Using cv2.erode() method 
eroded = cv2.erode(mask, kernel) 

# calculate moments of binary image
M = cv2.moments(eroded)

# calculate x,y coordinate of center
cX_origin = int(M["m10"] / M["m00"])
cY_origin = int(M["m01"] / M["m00"])

# Setup mmap exchange shape
shape1 = (480, 640, 3)
n1 = (480*640*3)

shape2 = (1080, 1920, 3)
n2 = (1080*1920*3)

# Setup mmap exchange destination
fd1 = os.open('mmaptest/test.png', os.O_RDWR)
#os.write(fd, b'\x00' * n)  # resize file
os.truncate(fd1, n1)  # resize file

fd2 = os.open('mmaptest/test1.png', os.O_RDWR)
#os.write(fd, b'\x00' * n)  # resize file
os.truncate(fd2, n2)  # resize file

# Initial mmap state
mm = None

# Open last opened frame from webcam
ret2, frame2 = cap.read()
# if not ret2:
#     break
if mm is None:
    mm2 = mmap.mmap(fd2, n2, mmap.MAP_SHARED, mmap.PROT_WRITE)  # it has to be only for writing
buf = frame2.tobytes()
mm2.seek(0)
mm2.write(buf)
mm2.flush()
# mm2.close()
cap.release()

# Initial fps setup
prev_ftime = 0
new_ftime = 0

# Detection thresholding
THRESHOLD = 0.3

while True: 
    # Read frame
    ret1, frame1 = cap1.read()

    ret2, frame2 = cap.read()
    if not ret2:
        break

    # cv2.imshow("coba",frame2)
    # cv2.waitKey(0)

    # Write image if frame is True
    if mm is None:
        mm1 = mmap.mmap(fd1, n1, mmap.MAP_SHARED, mmap.PROT_WRITE)  # it has to be only for writing
        mm2 = mmap.mmap(fd2, n2, mmap.MAP_SHARED, mmap.PROT_WRITE)  # it has to be only for writing
    
    buf = frame1.tobytes()
    mm1.seek(0)
    mm1.write(buf)
    mm1.flush()

    buf = frame2.tobytes()
    mm2.seek(0)
    mm2.write(buf)
    mm2.flush()

    image_np = np.array(frame1)
    
    input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.float32)
    detections = detect_fn(input_tensor)
    
    num_detections = int(detections.pop('num_detections'))
    detections = {key: value[0, :num_detections].numpy() for key, value in detections.items()}
    detections['num_detections'] = num_detections

    # detection_classes should be ints.
    detections['detection_classes'] = detections['detection_classes'].astype(np.int64)

    label_id_offset = 1
    image_np_with_detections = image_np.copy()

    viz_utils.visualize_boxes_and_labels_on_image_array(
                image_np_with_detections,
                detections['detection_boxes'],
                detections['detection_classes']+label_id_offset,
                detections['detection_scores'],
                category_index,
                use_normalized_coordinates=True,
                max_boxes_to_draw=10,
                min_score_thresh=THRESHOLD,
                agnostic_mode=False)

    polly = cv2.fillPoly(frame1, pts = [contours], color =(255,255,255))
    mask = cv2.inRange(polly,lower_color_bounds,upper_color_bounds )
    mask_rgb = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
    sway = frame1 & mask_rgb

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

    font = cv2.FONT_HERSHEY_SIMPLEX
    
    new_ftime = time.time()
    fps = 1/(new_ftime - prev_ftime)
    prev_ftime = new_ftime
    fps = str(int(fps))
    
    cv2.putText(image_np_with_detections, fps, (10,30), font, 0.5, (100, 255, 0), 1, cv2.LINE_AA)

    ctr = np.zeros(len(detections))
    
    for i in range(len(detections)):
        if detections['detection_scores'][i]>=THRESHOLD:
            ymin, xmin, ymax, xmax = detections['detection_boxes'][i]
            x_center = int((xmin+(xmax-xmin)/2)*width1)
            y_center = int((ymin+(ymax-ymin)/2)*height1)
            #x_center = int(width/2)
            #y_center = int(height/2)
            #print("Bounding Box: {}".format(detections['detection_boxes'][i]))
            #print("Center: {},{}".format(x_center, y_center))
            ctr[i] = x_center
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cap1.release()
                break
            

            cv2.circle(image_np_with_detections, (x_center, y_center), 10, (100, 255, 0), 3)

    # Select shortest detected distance
    ctr = ctr[ctr != 0.0]
    dctr = abs(width1/2 - ctr)
    center = ctr[np.argmin(dctr)]

    # Publish center of detected object        
    rospy.loginfo(center)  
    pub1.publish(int(center))

    #cv2.imshow('object detection',  cv2.resize(image_np_with_detections, (960, 720)))
    cv2.imshow('object detection',  image_np_with_detections)

    # Publish sway
    rospy.loginfo(cX-cX_origin)  
    pub2.publish(int(cX-cX_origin))

    # cv2.imshow('mask',mask)
    cv2.imshow('eroded',eroded)

    # cv2.imshow('camera 2', frame2)

    # rospy.spin()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap1.release()
        cap.release()
        mm1.close()
        mm2.close()
        break
# cap1.release()
# cap.release()
# mm1.close()
# mm2.close()

