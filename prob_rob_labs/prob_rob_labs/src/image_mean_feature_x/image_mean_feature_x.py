import rclpy
import numpy
from rclpy.node import Node
from std_msgs.msg import Float64
from prob_rob_msgs.msg import Point2DArrayStamped

class ImageMeanFeatureX(Node):

    def __init__(self):
        super().__init__('image_mean_feature_x')
        self.log = self.get_logger()
        self.sub_features = self.create_subscription(
            Point2DArrayStamped, '/goodfeature/corners',
            self.handle_features, 1)
        self.pub_mean = self.create_publisher(
            Float64, '/feature_mean', 1)

    def spin(self):
        rclpy.spin(self)

    def handle_features(self, f):
        x_features = [ p.x for p in f.points ]
        mean_x = numpy.mean(x_features)
        self.pub_mean.publish(Float64(data=mean_x))

def main():
    rclpy.init()
    image_mean_feature_x = ImageMeanFeatureX()
    image_mean_feature_x.spin()
    image_mean_feature_x.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
