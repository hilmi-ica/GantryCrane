# PATH Setup (Change CHECKPOINT_PATH and CUSTOM_MODEL_NAME)
ANNOTATION_PATH = "annotations"
MODEL_PATH = "models"
CHECKPOINT_PATH = MODEL_PATH+'/SSDMobileNet_Container_160821/'
CUSTOM_MODEL_NAME = "SSDMobileNet_Container_160821"
CONFIG_PATH = MODEL_PATH + '/' + CUSTOM_MODEL_NAME + '/pipeline.config'

# Import Tensorflow Module and Tensorflow Object Detection API
import tensorflow as tf
from object_detection.utils import config_util
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils
from object_detection.builders import model_builder

# Import Operating System Module (access file etc)
import os

# Import time (calculate fps, sampling time, etc)
import time

# Import Rospy Module (ROS MQTT (Pub/Sub) Communication)
import rospy
from std_msgs.msg import UInt16

# Import OpenCV and Numpy (Image processing)
import cv2 
import numpy as np

###########
# Setup ROS
###########

# Setup ROS Node, Publisher, and Callback
pub = rospy.Publisher('object_point', UInt16, queue_size=10)
rospy.init_node('camera_node', anonymous=False)

###########
# Setup Tensorflow
###########

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
cap = cv2.VideoCapture(2) # Change camera index if needed
cap.release() # Prevent previous unreleased session
cap = cv2.VideoCapture(2) # Change camera index if needed
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Setup class for storing variable
# IDK why this is the most robust method in this case :v
class variable:
    def __init__(self,mode):
        self.mode = mode

var = variable(mode=0)


# Setup Talker
def talker():
    # Setup for fps
    prev_ftime = 0
    new_ftime = 0

    # Detection threshold
    THRESHOLD = 0.3

    # Frame loop
    while True: 
        # Read frame
        ret, frame = cap.read()
        image_np = np.array(frame)
        
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
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Detectoion fps
        new_ftime = time.time()
        fps = 1/(new_ftime - prev_ftime)
        prev_ftime = new_ftime
        fps = str(int(fps))
        
        cv2.putText(image_np_with_detections, fps, (10,30), font, 0.5, (100, 255, 0), 1, cv2.LINE_AA)
        
        for i in range(len(detections)):
            if detections['detection_scores'][i]>=THRESHOLD:
                ymin, xmin, ymax, xmax = detections['detection_boxes'][i]
                x_center = int((xmin+(xmax-xmin)/2)*width)
                y_center = int((ymin+(ymax-ymin)/2)*height)
                #x_center = int(width/2)
                #y_center = int(height/2)
                #print("Bounding Box: {}".format(detections['detection_boxes'][i]))
                #print("Center: {},{}".format(x_center, y_center))
                # Publish center of detection box
                rospy.loginfo(x_center)  
                pub.publish(int(x_center))
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cap.release()
                    break
                
                # Circle at center of detection box
                cv2.circle(image_np_with_detections, (x_center, y_center), 10, (100, 255, 0), 3)

        # print(x_center)   
        # pub(int(x_center))

        # Show frame
        #cv2.imshow('object detection',  cv2.resize(image_np_with_detections, (960, 720)))
        cv2.imshow('object detection',  image_np_with_detections)
    

        # rospy.spin()
        
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