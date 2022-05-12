
from perception_interfaces.srv import ObjectPoses
import rclpy
from rclpy.node import Node


class PerceptionClient(Node):
    def __init__(self):
        super().__init__('perception_client')
        self._perception_client = self.create_client(ObjectPoses, 'object_poses')

        while not self._perception_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for perception service')

    def send_request(self):
        req = ObjectPoses.Request()
        req.objects = ['astro_boy']
        self.future = self._perception_client.call_async(req)
  
    def get_pose(self):
        if self.future.done():
            try:
                response =  self.future.result()
            except Exception as e:
                self.get_logger().info('Perception Request Failed')
                return []
            else:
                if not response.poses:
                    self.get_logger().info('Object Found')

                self.get_logger().info('Perception Request Successful')
                return response.poses
        else:
            return None


def main(args=None):
    rclpy.init(args=args)

    perception_client = PerceptionClient()
    perception_client.send_request()

    rclpy.spin_once(perception_client)
    poses = perception_client.get_pose()
  
    perception_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



