import os, json, pika, rclpy
from std_msgs.msg import String
from rclpy.node import Node

class Bridge(Node):
    def __init__(self):
        super().__init__('queue_bridge')
        self.pub = self.create_publisher(String, '/fleet/tasks', 10)

def main():
    rclpy.init()
    node = Bridge()

    url = os.getenv("RABBITMQ_URL", "amqp://guest:guest@rabbitmq:5672/")
    params = pika.URLParameters(url)
    conn = pika.BlockingConnection(params)
    ch = conn.channel()
    ch.queue_declare(queue='fleet_tasks', durable=True)

    def cb(ch_, method, properties, body):
        node.get_logger().info(f"Task from queue: {body}")
        node.pub.publish(String(data=body.decode('utf-8')))
        ch_.basic_ack(delivery_tag=method.delivery_tag)

    ch.basic_qos(prefetch_count=1)
    ch.basic_consume(queue='fleet_tasks', on_message_callback=cb)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        conn.close()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
