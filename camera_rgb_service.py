#!/usr/bin/env python3
"""
相机 RGB 数据服务节点
提供 4 个相机的 RGB 图像数据服务
严格按照 RealSenseRGBImage.srv 协议格式
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from jqr_ros_msgs.srv import RealSenseRGBImage


class CameraRGBService(Node):
    """相机 RGB 数据服务节点"""

    # 支持的相机 ID
    CAMERA_IDS = ["cameraF", "cameraB", "cameraL", "cameraR"]

    # 图像文件目录（相对于运行脚本的位置）
    IMAGE_DIR = "camera_images"

    # 图像文件名映射
    IMAGE_FILES = {
        "cameraF": "1.jpg",
        "cameraB": "2.jpg",
        "cameraL": "6.jpg",
        "cameraR": "8.jpg"
    }

    def __init__(self):
        super().__init__('camera_rgb_service')

        # 创建服务
        self.srv = self.create_service(
            RealSenseRGBImage,
            '/realsense_rgb_image',
            self.get_rgb_images_callback
        )

        self.get_logger().info('相机 RGB 数据服务已启动')
        self.get_logger().info(f'支持相机 ID: {self.CAMERA_IDS}')
        self.get_logger().info(f'图像目录: {os.path.abspath(self.IMAGE_DIR)}')

        # 检查图像文件是否存在
        self._check_image_files()

    def _check_image_files(self):
        """检查图像文件是否存在"""
        missing_files = []
        for camera_id, filename in self.IMAGE_FILES.items():
            filepath = os.path.join(self.IMAGE_DIR, filename)
            if not os.path.exists(filepath):
                missing_files.append(filepath)

        if missing_files:
            self.get_logger().warning('以下图像文件不存在:')
            for f in missing_files:
                self.get_logger().warning(f'  - {f}')
            self.get_logger().warning('请确保图像文件存在，否则服务将返回空数据')
        else:
            self.get_logger().info('所有相机图像文件检查通过')

    def _load_image(self, camera_id: str) -> bytes:
        """加载指定相机的图像数据

        Args:
            camera_id (str): 相机 ID

        Returns:
            bytes: JPEG 格式的图像数据
        """
        try:
            filename = self.IMAGE_FILES.get(camera_id)
            if not filename:
                self.get_logger().error(f'未知的相机 ID: {camera_id}')
                return b''

            filepath = os.path.join(self.IMAGE_DIR, filename)

            if not os.path.exists(filepath):
                self.get_logger().warning(f'图像文件不存在: {filepath}')
                return b''

            with open(filepath, 'rb') as f:
                image_data = f.read()

            self.get_logger().debug(f'成功加载相机 {camera_id} 的图像，大小: {len(image_data)} bytes')
            return image_data

        except Exception as e:
            self.get_logger().error(f'加载相机 {camera_id} 的图像失败: {e}')
            return b''

    def _create_compressed_image(self, camera_id: str) -> CompressedImage:
        """创建压缩图像消息

        Args:
            camera_id (str): 相机 ID

        Returns:
            CompressedImage: 压缩图像消息
        """
        # 加载图像数据
        image_data = self._load_image(camera_id)

        # 创建 CompressedImage 消息
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = camera_id
        msg.format = "jpeg"
        msg.data = image_data

        return msg

    def get_rgb_images_callback(self, request, response):
        """获取 RGB 图像服务回调

        Args:
            request: RealSenseRGBImage.Request
            response: RealSenseRGBImage.Response

        Returns:
            RealSenseRGBImage.Response
        """
        self.get_logger().info(f'收到获取 RGB 图像请求，请求相机数: {len(request.camera_ids)}')

        # 验证请求的相机 ID
        valid_camera_ids = []
        for camera_id in request.camera_ids:
            if camera_id in self.CAMERA_IDS:
                valid_camera_ids.append(camera_id)
            else:
                self.get_logger().warning(f'不支持的相机 ID: {camera_id}，已跳过')

        if not valid_camera_ids:
            self.get_logger().error('请求中没有有效的相机 ID')
            return response

        self.get_logger().info(f'处理相机: {valid_camera_ids}')

        # 为每个相机加载图像
        for camera_id in valid_camera_ids:
            compressed_img = self._create_compressed_image(camera_id)
            response.camera_ids.append(camera_id)
            response.rgb_images_compressed.append(compressed_img)

            if compressed_img.data:
                self.get_logger().debug(f'相机 {camera_id} 图像大小: {len(compressed_img.data)} bytes')
            else:
                self.get_logger().warning(f'相机 {camera_id} 返回空图像')

        self.get_logger().info(
            f'返回 {len(response.camera_ids)} 个相机的图像数据'
        )

        return response


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    camera_rgb_service = CameraRGBService()

    try:
        rclpy.spin(camera_rgb_service)
    except KeyboardInterrupt:
        pass
    finally:
        camera_rgb_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
