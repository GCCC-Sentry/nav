FROM ros:humble-ros-base

# 1. 安装基础工具
# 这里的 apt update 可能会因为网络问题偶尔失败，建议保留
RUN sudo apt update && \
    sudo apt install -y python3-pip curl wget htop vim unzip libeigen3-dev libomp-dev && \
    pip install xmacro gdown

# 2. (可选) 设置 zsh
# 如果之前这步卡住，保持注释状态即可；如果想尝试，可以取消注释
# RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
#     -t jispwoso -p git \
#     -p https://github.com/zsh-users/zsh-autosuggestions \
#     -p https://github.com/zsh-users/zsh-syntax-highlighting && \
#     chsh -s /bin/zsh
CMD [ "/bin/bash" ]

# -------------------------------------------------------
# 3. 安装 small_gicp (关键修改：使用本地 COPY 替代 git clone)
# -------------------------------------------------------
# 将宿主机当前目录下的 small_gicp 文件夹复制到容器的 /tmp/small_gicp
COPY small_gicp /tmp/small_gicp

# 编译并安装
RUN cd /tmp/small_gicp && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/small_gicp

# -------------------------------------------------------
# 4. 复制本地项目代码
# -------------------------------------------------------
WORKDIR /root/ros_ws
# 将当前目录下的所有文件（除了 .dockerignore 排除的）复制到容器源码目录
COPY . src/pb2025_sentry_nav/

# 5. 安装依赖 (rosdep)
RUN rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 6. 下载仿真 PCD 文件
# 如果这一步 gdown 也因为网络报错，可以手动下载文件放在对应目录，然后注释掉这块
# RUN gdown https://drive.google.com/uc\?id\=1kAxdOU-mi1TcLssyKmDR0pz0ojF96jQ4 -O /tmp/simulation_pcd.zip && \
#     unzip -o /tmp/simulation_pcd.zip -d /root/ros_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/ && \
#     rm /tmp/simulation_pcd.zip

# 7. 预编译 (可选)
# 使用 bash 执行，确保环境变量生效
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# 8. 环境设置
# 写入 .bashrc (因为我们上面禁用了 zsh，这里改写 bashrc 更通用)
RUN echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> /root/.bashrc && \
    echo 'source /root/ros_ws/install/setup.bash' >> /root/.bashrc

# 9. 修正入口点
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

# 清理缓存
RUN rm -rf /var/lib/apt/lists/*