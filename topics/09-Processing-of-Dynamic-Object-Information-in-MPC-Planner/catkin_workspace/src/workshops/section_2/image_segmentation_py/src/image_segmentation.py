#!/usr/bin/env python

#
#  ==============================================================================
#  MIT License
#
#  Copyright 2022 Institute for Automotive Engineering of RWTH Aachen University.
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
#  ==============================================================================
#

import rospy
import numpy as np
from sensor_msgs import msg

import cv2
from cv_bridge import CvBridge

import time
import os
import img_utils
import xml.etree.ElementTree as ET


WITH_TF = True
try:
    import tensorflow as tf
except:
    rospy.logwarn("%s will shutdown because it was not compiled with TensorFlow", rospy.get_name())
    WITH_TF = False



class ImageSegmentation:


    def predict(self, img_rect_color_msg):

        t0 = time.time()

        # convert message to cv2-image
        input_img = self.cv_bridge.imgmsg_to_cv2(img_rect_color_msg, desired_encoding="rgb8")

        # resize image
        input_img = img_utils.resize_image(input_img, [self.resize_height, self.resize_width])
        
        # append batch dimension
        input_img = input_img[None]
        t1 = time.time()

        # perform semantic segmentation
        predictions = self.frozen_func(tf.cast(input_img, tf.uint8))

        t2 = time.time()

        # remove batch dimension
        prediction = tf.squeeze(predictions).numpy()

        # decode image to RGB
        prediction = self.segmentation_map_to_rgb(prediction).astype(np.uint8)

        # convert output back to message
        seg_msg = self.cv_bridge.cv2_to_imgmsg(prediction, encoding="rgb8")

        # assign header of input msg
        seg_msg.header = img_rect_color_msg.header
        
        t3 = time.time()

        # log processing duration
        rospy.loginfo("(prep) %fs | (pred) %fs | (post) %fs | (total) %fs", t1-t0, t2-t1, t3-t2, t3-t0)


        self.pub_seg.publish(seg_msg)


    @staticmethod
    def wrap_frozen_graph(graph_def, inputs, outputs, print_graph=False):
        def _imports_graph_def():
            tf.compat.v1.import_graph_def(graph_def, name="")

        wrapped_import = tf.compat.v1.wrap_function(_imports_graph_def, [])
        import_graph = wrapped_import.graph

        return wrapped_import.prune(
            tf.nest.map_structure(import_graph.as_graph_element, inputs),
            tf.nest.map_structure(import_graph.as_graph_element, outputs))


    def load_frozen_graph(self, path_to_frozen_graph):
        self.sess = None
        self.graph = tf.Graph()

        self.input_tensor_name = 'input:0'
        self.output_tensor_name = 'prediction:0'

        with tf.io.gfile.GFile(path_to_frozen_graph, 'rb') as file_handle:
            graph_def = tf.compat.v1.GraphDef()
            loaded = graph_def.ParseFromString(file_handle.read())

        # Wrap frozen graph to ConcreteFunctions
        self.frozen_func = self.wrap_frozen_graph(graph_def=graph_def,
                                                  inputs=[self.input_tensor_name],
                                                  outputs=[self.output_tensor_name],
                                                  print_graph=True)


    def segmentation_map_to_rgb(self, segmentation_map):
        """
        Converts segmentation map to a RGB encoding according to self.color_palette
        Eg. 0 (Class 0) -> Pixel value [128, 64, 128] which is on index 0 of self.color_palette
            1 (Class 1) -> Pixel value [244, 35, 232] which is on index 1 of self.color_palette

        self.color_palette has shape [256, 3]. Each index of the first dimension is associated
        with an RGB value. The index corresponds to the class ID.

        :param segmentation_map: ndarray numpy with shape (height, width)
        :return: RGB encoding with shape (height, width, 3)
        """

        ### START CODE HERE ###
        
        # Task 1:
        # Replace the following command
        rgb_encoding = self.color_palette[segmentation_map]

        ### END CODE HERE ###
        return rgb_encoding


    def parse_convert_xml(self, conversion_file_path):
        """
        Parse XML conversion file and compute color_palette 
        """

        defRoot = ET.parse(conversion_file_path).getroot()

        color_to_label = {}

        color_palette = np.zeros((256, 3), dtype=np.uint8)
        class_list = np.ones((256), dtype=np.uint8) * 255
        class_names = np.array(["" for _ in range(256)], dtype='<U25')
        for idx, defElement in enumerate(defRoot.findall("SLabel")):
            from_color = np.fromstring(defElement.get("fromColour"), dtype=int, sep=" ")
            to_class = np.fromstring(defElement.get("toValue"), dtype=int, sep=" ")
            class_name = defElement.get('Name').lower()
            if to_class in class_list:
                color_to_label[tuple(from_color)] = int(to_class)
            else:
                color_palette[idx] = from_color
                class_list[idx] = to_class
                class_names[idx] = class_name
                color_to_label[tuple(from_color)] = int(to_class)

        # Sort classes accoring to is train ID
        sort_indexes = np.argsort(class_list)

        class_list = class_list[sort_indexes]
        class_names = class_names[sort_indexes]
        color_palette = color_palette[sort_indexes]

        return color_palette, class_names, color_to_label


    def load_parameters(self):
        """
        Load ROS parameters and store them
        """
        prefix = "image_segmentation/"

        rospy.loginfo("Loading parameters ...")

        package_dir = os.path.join(os.path.dirname(__file__), os.pardir)

        self.frozen_graph               = os.path.join(package_dir, rospy.get_param(prefix + "frozen_graph"))
        self.path_xml_conversion_file   = os.path.join(package_dir, rospy.get_param(prefix + "xml_conversion_file"))
        self.resize_width               = rospy.get_param(prefix + "resize_width", default=1936)
        self.resize_height              = rospy.get_param(prefix + "resize_height", default=1216)


        # self load one hot encoding
        self.color_palette, self.class_names, self.color_to_label = self.parse_convert_xml(self.path_xml_conversion_file)


    def setup(self):

        # create cv2-msg converter bridge
        self.cv_bridge = CvBridge()

        # load frozen graph
        self.load_frozen_graph(self.frozen_graph)

        # create publisher for passing on depth estimation and camera info
        self.pub_seg = rospy.Publisher("/image_rect_segmented", msg.Image, queue_size=1)

        # listen for input image and camera info
        self.sub_image = rospy.Subscriber("/image_rect_color", msg.Image, self.predict, queue_size=1, buff_size=50) # buff_size = 500 MB


    def __init__(self):

        # initialize ROS node
        rospy.init_node("camera_segmentation", anonymous=True)

        if WITH_TF:

            # load parameters
            self.load_parameters()

            # setup components
            self.setup()

            # keep node from exiting
            rospy.spin()


if __name__ == "__main__":

    vision = ImageSegmentation()
