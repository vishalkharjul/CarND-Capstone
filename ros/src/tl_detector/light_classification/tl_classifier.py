from styx_msgs.msg import TrafficLight 
import rospy
import argparse
import sys

import numpy as np
import tensorflow as tf

def load_graph(model_file):
    graph = tf.Graph()
    graph_def = tf.GraphDef()

    with open(model_file, "rb") as f:
        graph_def.ParseFromString(f.read())
    with graph.as_default():
        tf.import_graph_def(graph_def)

    return graph

def load_labels(label_file):
    label = []
    proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
    for l in proto_as_ascii_lines:
        label.append(l.rstrip())
    return label

def read_tensor_from_image(image, input_height=224, input_width=224,
                input_mean=0, input_std=255):
  
    float_caster = tf.cast(image, tf.float32)
    dims_expander = tf.expand_dims(float_caster, 0);
    resized = tf.image.resize_bilinear(dims_expander, [input_height, input_width])
    normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])
    sess = tf.Session()
    result = sess.run(normalized)

    return result

class TLClassifier(object):
    def __init__(self, carla_run):

		self.carla_run = carla_run
		
		model_path="models/sim/retrained_graph.pb"
		labels_path="models/sim/image_labels.txt"
		if self.carla_run:
			model_path="models/carla/output_graph.pb"
			labels_path="models/carla/output_labels.txt"
		
		self.labels=load_labels(labels_path)		
		self.graph = load_graph(model_path)
		rospy.loginfo("Graph %s loaded", model_path)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        #TODO implement light color prediction
        t = read_tensor_from_image(image)
        
        input_name = "import/input"
        output_name = "import/final_result"
        input_operation = self.graph.get_operation_by_name(input_name)
        output_operation = self.graph.get_operation_by_name(output_name)


        with tf.Session(graph=self.graph) as sess:
                results = sess.run(output_operation.outputs[0],
                          {input_operation.outputs[0]: t})
        results = np.squeeze(results)
        top_k = results.argsort()[-5:][::-1]

        ret=TrafficLight.UNKNOWN
        first = top_k[0]
        if results[first] > .8:
            if self.labels[first] == "red" or self.labels[first] == "stop":
                ret = TrafficLight.RED
            elif self.labels[first] == "green" or self.labels[first] == "go":
                ret = TrafficLight.GREEN
        rospy.logdebug("The detected signal is: %s", self.labels[first])
        return ret