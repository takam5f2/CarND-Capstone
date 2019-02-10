from styx_msgs.msg import TrafficLight

import numpy as np
import os
import sys
import tensorflow as tf

from collections import defaultdict
from io import StringIO
from matplotlib import pyplot as plt
from PIL import Image
import cv2

sys.path.append("..")
from object_detection.utils import ops as utils_ops

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from helper_classifier import load_image_into_numpy_array
from helper_classifier import run_inference_for_single_image

import rospy
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        cwd = os.getcwd()
        PATH_TO_FROZEN_GRAPH = cwd + '/light_classification/models/graph/frozen_inference_graph.pb'

        PATH_TO_LABELS = cwd + '/light_classification/models/bstld_label_map.pbtxt'

        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

                self.category_index = label_map_util.create_category_index_from_labelmap(PATH_TO_LABELS)


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_np = load_image_into_numpy_array(image)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        output_dict = run_inference_for_single_image(image_np, self.detection_graph)
        
        num = output_dict['num_detections']
        classes =  output_dict['detection_classes']
        detection_scores =  output_dict['detection_scores']
        # 
        max_scores = 0.0
        tl_state = TrafficLight.UNKNOWN

        for i in range(num):
            if max_scores < detection_scores[i]:
                max_scores = detection_scores[i]
                str_tl_state = "Unknown"
                if classes[i] == 1:
                    tl_state = TrafficLight.UNKNOWN
                elif classes[i] == 2:
                    tl_state = TrafficLight.GREEN
                    str_tl_state = "Green"
                elif classes[i] == 3:
                    tl_state = TrafficLight.YELLOW
                    str_tl_state = "Yellow"
                elif classes[i] == 4:
                    tl_state = TrafficLight.RED
                    str_tl_state = "Red"
                else:
                    tl_state = TrafficLight.UNKNOWN
            sys.stderr.write("----- traffic light : {}({}) ------\n".format(str_tl_state, tl_state))
        
                        
        #TODO implement light color prediction
        return tl_state
