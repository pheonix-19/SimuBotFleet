FROM osrf/ros:jazzy-desktop-full

# Install Python dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
SHELL ["/bin/bash", "-c"]

WORKDIR /app
COPY requirements.txt .

# Install Python packages (excluding rclpy since it's provided by ROS)
RUN pip install --no-cache-dir --break-system-packages \
    fastapi \
    uvicorn[standard] \
    pydantic \
    celery \
    redis \
    pika \
    psycopg2-binary \
    prometheus-client

COPY app/ app/

ENV PYTHONUNBUFFERED=1
EXPOSE 8000

# Source ROS and start the application
CMD ["bash", "-c", "source /opt/ros/jazzy/setup.bash && uvicorn app.main:app --host 0.0.0.0 --port 8000"]
