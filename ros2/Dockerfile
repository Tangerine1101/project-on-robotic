# Sử dụng base image ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop

# Thiết lập biến môi trường để tránh hỏi tương tác khi cài đặt
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV PIP_ROOT_USER_ACTION=ignore

# 1. Cài đặt các công cụ hệ thống và thư viện bổ trợ cho OpenCV/YOLO
RUN apt-get update && apt-get install -y \
    nano \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 2. Xử lý Numpy TRƯỚC TIÊN (Quan trọng nhất)
# ROS 2 Humble KHÔNG tương thích với Numpy 2.0.x (có trong file requirements của bạn)
# Bắt buộc phải cài bản < 2.0
RUN pip3 install "numpy<2.0"

# 3. Cài đặt YOLO11 (Ultralytics) và các thư viện cần thiết
# 'ultralytics' sẽ tự động cài PyTorch, OpenCV, Matplotlib phù hợp
# Chúng ta cài thêm các thư viện từ requirements.txt của bạn (đã lọc sạch)
RUN pip3 install --no-cache-dir \
    ultralytics \
    pyserial \
    psutil \
    redis \
    lxml \
    polars-lts-cpu \
    pyarrow \
    pandas \
    scipy \
    requests \
    PyYAML

# 4. Thiết lập môi trường cho VS Code
# Tự động source ROS khi mở Terminal trong VS Code
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 5. Cấu hình Entrypoint
# Copy file entrypoint từ máy host vào container
COPY ./build/ros_entrypoint.sh /ros_entrypoint.sh
# Cấp quyền thực thi cho script
RUN chmod +x /ros_entrypoint.sh

# Thiết lập Entrypoint để luôn source ROS khi khởi động container
ENTRYPOINT ["/ros_entrypoint.sh"]

# 


# Lệnh mặc định khi container chạy
CMD ["bash"]