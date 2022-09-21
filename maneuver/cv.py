# Import Operating System
import os

# Import Computer Vision
import cv2

# Import mmap (image exchange)
import mmap

# Import Pytorch (help depth estimation)
import torch 

# Import Numpy
import numpy as np

# Import difflib (Matching text)
from difflib import SequenceMatcher

# Import Tesseract OCR (text recognition)
import pytesseract

# Import ROS Packages
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

# Import Pytorch packages
from torchvision.transforms import Compose
from midas.dpt_depth import DPTDepthModel
from midas.midas_net_custom import MidasNet_small
from midas.transforms import Resize, NormalizeImage, PrepareForNet

# Import independent built packages
import frenet
import frenet1
import window
import window1
import vacancy
import utils

# Capture image or video
def capture(i=2, set=False, x=640, y=480):
    cap = cv2.VideoCapture(i)
    if set:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, x)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, y)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # realtime
    while True:
        ret,frame = cap.read()

        cv2.imshow('webcam',cv2.resize(frame, (width,height)))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cap.release()
            break
    cap.release()

    return frame

# Client of image exchange from mmap
def client(file=0, x=640, y=480):
    if file == 0:
        filename = 'mmaptest/test.png'
    elif file == 1:
        filename = 'mmaptest/test1.png'
    shape = (y, x, 3)
    n = (y*x*3)

    fd = os.open(filename, os.O_RDONLY)

    mm = mmap.mmap(fd, n, mmap.MAP_SHARED, mmap.PROT_READ)  # it has to be only for reading

    # while True:
    #     # read image
    #     mm.seek(0)
    #     buf = mm.read(n)
    #     img = np.frombuffer(buf, dtype=np.uint8).reshape(shape)

    #     cv2.imshow("client", img)
    #     key = cv2.waitKey(1) & 0xFF
    #     key = chr(key)
    #     if key.lower() == "q":
    #         break
        
    mm.seek(0)
    buf = mm.read(n)
    img = np.frombuffer(buf, dtype=np.uint8).reshape(shape)

    cv2.destroyAllWindows()
    mm.close()

    return img

# Text recognition
def text(i=2, x=640, y=480, stack=0.25):
    # Image capture
    # frame = capture(i)
    frame = client(0,x,y)
    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

    # Image processing based on location of the text
    if round(stack,3) == 0.25:
        # cropped = frame[250:275,270:345]
        cropped = frame[235:290,255:360]
        cropped = cv2.resize(cropped,None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC)
        cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
        cropped = cv2.dilate(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.erode(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.medianBlur(cropped,5)
        cropped = cv2.threshold(cropped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    elif round(stack,3) == 0.39:
        # cropped = frame[255:285,265:360]
        cropped = frame[240:300,250:375]
        cropped = cv2.resize(cropped,None,fx=7,fy=7,interpolation=cv2.INTER_CUBIC)
        cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
        cropped = cv2.dilate(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.erode(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.medianBlur(cropped,7)
        cropped = cv2.threshold(cropped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    elif round(stack,3) == 0.52:
        # cropped = frame[260:300,260:375]
        cropped = frame[245:315,245:390]
        cropped = cv2.resize(cropped,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC)
        cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
        cropped = cv2.dilate(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.erode(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.medianBlur(cropped,3)
        cropped = cv2.threshold(cropped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
    elif round(stack,3) == 0.64:
        # cropped = frame[270:320,245:395]
        cropped = frame[255:335,230:410]
        cropped = cv2.resize(cropped,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC)
        cropped = cv2.cvtColor(cropped,cv2.COLOR_BGR2GRAY)
        cropped = cv2.dilate(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.erode(cropped, np.ones((5, 5), np.uint8), iterations=1)
        cropped = cv2.medianBlur(cropped,9)
        cropped = cv2.threshold(cropped, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

    cv2.imshow("text",cropped)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Recognition
    text = []
    boxes = pytesseract.image_to_data(cropped)
    # boxes = pytesseract.image_to_data(frame)
    for x,b in enumerate(boxes.splitlines()):
        if x != 0:
            b = b.split()
            if len(b) == 12:
                text.append(b[11])
                print(b[11])

    text = ' '.join(word for word in text)

    # Sequence matching
    serials = np.loadtxt('id.csv',str,delimiter=',')
    ratio = []
    for idx, serial in enumerate(serials[1:,0]):
        ratio.append(SequenceMatcher(None,serial,text).ratio())
    best_idx = [index for index, item in enumerate(ratio) if item == max(ratio)]

    # Debugging for same ratio result
    if len(best_idx) >= 1:
        best_idx = best_idx[0]
    best_idx = int("".join(list(map(str,best_idx))))
    print(serials[best_idx+1,0])

# Depth estimation
def depth(i=4, model_type="large", optimize=True):
    # Setup ROS Node, Publisher, and Callback
    # pub = rospy.Publisher('depth', Float32MultiArray, queue_size=10)
    # rospy.init_node('camera_node', anonymous=False)

    # select device
    # device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = torch.device("cpu")
    print("device: %s" % device)

    # load network
    if model_type == "large": # DPT-Large
        model_path = "models/dpt_large-midas-2f21e586.pt"
        model = DPTDepthModel(
            path=model_path,
            backbone="vitl16_384",
            non_negative=True,
        )
        net_w, net_h = 384, 384
        resize_mode = "minimal"
        normalization = NormalizeImage(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    elif model_type == "hybrid": #DPT-Hybrid
        model_path = "models/dpt_hybrid-midas-501f0c75.pt"
        model = DPTDepthModel(
            path=model_path,
            backbone="vitb_rn50_384",
            non_negative=True,
        )
        net_w, net_h = 384, 384
        resize_mode="minimal"
        normalization = NormalizeImage(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])
    elif model_type == "midas_v21_small":
        model_path = "models/midas_v21_small-70d6b9c8.pt"
        model = MidasNet_small(model_path, features=64, backbone="efficientnet_lite3", exportable=True, non_negative=True, blocks={'expand': True})
        net_w, net_h = 256, 256
        resize_mode="upper_bound"
        normalization = NormalizeImage(
            mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
        )
    else:
        print(f"model_type '{model_type}' not implemented, use: --model_type large")
        assert False

    transform = Compose(
        [
            Resize(
                net_w,
                net_h,
                resize_target=None,
                keep_aspect_ratio=True,
                ensure_multiple_of=32,
                resize_method=resize_mode,
                image_interpolation_method=cv2.INTER_CUBIC,
            ),
            normalization,
            PrepareForNet(),
        ]
    )

    model.eval()
    
    if optimize==True:
        # rand_example = torch.rand(1, 3, net_h, net_w)
        # model(rand_example)
        # traced_script_module = torch.jit.trace(model, rand_example)
        # model = traced_script_module
    
        if device == torch.device("cuda"):
            model = model.to(memory_format=torch.channels_last)  
            model = model.half()

    model.to(device)

    # input
    # img = capture(i,True,1920,1080)
    # img = client(1,1920,1080)
    img = cv2.flip(img,-1)
    img = utils.read_image(img)
    img_input = transform({"image": img})["image"]

    with torch.no_grad():
        sample = torch.from_numpy(img_input).to(device).unsqueeze(0)
        if optimize==True and device == torch.device("cuda"):
            sample = sample.to(memory_format=torch.channels_last)  
            sample = sample.half()
        prediction = model.forward(sample)
        prediction = (
            torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            )
            .squeeze()
            .cpu()
            .numpy()
        )
    depth = utils.write_depth(prediction, bits=2)
    depth_normalized = cv2.normalize(depth, None, 0, 1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)

    depth_metric = vacancy.convert1(depth_normalized)
    dist_cont = vacancy.distance(depth_metric)
    vac_cont = vacancy.vacancies(dist_cont)
    x_set, y, y_real = vacancy.locate(vac_cont)
    print(vac_cont,x_set,y,y_real)

    # img_magma = cv2.applyColorMap(depth_normalized,cv2.COLORMAP_MAGMA)
    cv2.imshow("depth",cv2.resize(depth_normalized,(1280,720)))
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return x_set, y, y_real, vac_cont, depth_normalized 

# Path planning
def path(sx, sy, gx, gy, vac, type='frenet', unit='m'): # Not finished or complete so not recommended
    # Frenet optimalization method
    if type == 'frenet':
        if unit == 'm':
            frenet.main(sx,sy,gx,gy,vac)
        elif unit == 'cm':
            frenet1.main(sx,sy,gx,gy,vac)
        else:
            print("Unit chosen is not listed!")
    
    # Dynamic window approach
    elif type == 'window':
        if unit == 'm':
            window.main(sx,sy,gx,gy,vac)
        elif unit == 'cm':
            window1.main(sx,sy,gx,gy,vac)
        else:
            print("Unit chosen is not listed!")
    else:
        print("Type chosen is not listed!")