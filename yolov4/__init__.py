import sys
import os
sys.path.append(os.getcwd() + '/yolov4')
#print(sys.path)

from tool.utils import *
from tool.torch_utils import *
from tool.darknet2pytorch import Darknet

def NewDarknet(use_tiny=True, use_cuda=False):
    if not use_tiny:
        # yolov4
        darknet = Darknet("./yolov4/cfg/yolov4.cfg")
        darknet.load_weights("./yolov4/weight/yolov4.weights")
    else:
        # yolov4-tiny
        darknet = Darknet("./yolov4/cfg/yolov4-tiny.cfg")
        darknet.load_weights("./yolov4/weight/yolov4-tiny.weights")

    if use_cuda:
        darknet.cuda()

    #darknet.print_network()

    num_classes = darknet.num_classes
    if num_classes == 20:
        namesfile = './yolov4/data/voc.names'
    elif num_classes == 80:
        namesfile = './yolov4/data/coco.names'
    else:
        namesfile = './yolov4/data/x.names'
    class_names = load_class_names(namesfile)

    return darknet, class_names
