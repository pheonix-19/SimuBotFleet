# For demo, publish to a simple RabbitMQ queue 'fleet_tasks'.
# A ROS 2 bridge subscriber reads and republishes to /fleet/tasks.
import os, pika

def _conn():
    url = os.getenv("RABBITMQ_URL", "amqp://guest:guest@rabbitmq:5672/")
    params = pika.URLParameters(url)
    return pika.BlockingConnection(params)

def enqueue_task(payload: str):
    conn = _conn()
    ch = conn.channel()
    ch.queue_declare(queue='fleet_tasks', durable=True)
    ch.basic_publish(exchange='', routing_key='fleet_tasks', body=payload.encode('utf-8'))
    conn.close()
