import sys

from rione_msgs.srv import RequestLocation
from rione_msgs.msg import Location
import rclpy
from rclpy.node import Node

from random import uniform

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(RequestLocation, '/location_register')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = RequestLocation.Request()

    def send_request(self):
        self.req.command = sys.argv[1]
        self.req.file = "carry_my_luggage"

        location = Location()
        location.name = "sample"
        location.position.x = uniform(-10, 10)
        location.position.y = uniform(-10, 10)
        location.position.z = uniform(-10, 10)
        location.contents.append("hello:world")
        location.contents.append("pub/sub")
        self.req.locations.append(location)
        #location = Location()
        #location.name = "test"
        #self.req.locations.append(location)

        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of REQUEST: for %s = %s' %
                    (minimal_client.req.command, response.flag))
                for location in response.locations:
                    print(location)
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
