#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes
from std_msgs.msg import Header
import numpy as np

filter = 2

class YOLODetector:
    def __init__(self):
        # 类型稳定性相关变量
        self.last_type = None
        self.stable_count = 0
        self.switch_pending = None
        self.no_type_count = 0
        
        # 初始化ROS节点
        rospy.init_node('yolo_detector', anonymous=True)
        
        # 获取模型路径参数
        weight_path = rospy.get_param('~weight_path', '../weights/endurance.pt')
        
        # 获取置信度参数
        conf = rospy.get_param('~conf', 0.6)
        
        # 获取设备参数
        use_cpu = rospy.get_param('~use_cpu', False)
        
        # 确定使用的设备
        if use_cpu:
            self.device = 'cpu'
        else:
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # 加载YOLOv8模型
        self.model = YOLO(weight_path)
        
        # 设置模型设备和置信度
        if torch.cuda.is_available() and not use_cpu:
            self.model.to(self.device)
        self.model.conf = conf
        
        # 初始化CvBridge
        self.bridge = CvBridge()
        
        # 获取图像话题参数
        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        
        # 获取发布话题参数
        pub_topic = rospy.get_param('~pub_topic', '/object_position')
        
        # 获取可视化参数
        self.visualize = rospy.get_param('~visualize', True)

        # 获取增强参数
        self.enhanced = rospy.get_param('~enhanced', True)
        
        # 获取图像发布话题参数
        image_pub_topic = rospy.get_param('~image_pub_topic', '/yolov8/detection_image')
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        # 发布边界框消息
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        
        # 发布检测结果图像
        self.image_pub = rospy.Publisher(image_pub_topic, Image, queue_size=1)
        
        # 帧率统计初始化
        self.last_time = rospy.Time.now()
        self.fps = 0.0
        self.printed_resolution = None
        
        # 打印启动信息
        rospy.loginfo(f"加载模型: {weight_path}")
        rospy.loginfo(f"使用设备: {self.device}")
        rospy.loginfo(f"CUDA可用: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            rospy.loginfo(f"CUDA设备名称: {torch.cuda.get_device_name(0)}")
        rospy.loginfo(f"推理设备: {'GPU (CUDA)' if self.device == 'cuda' else 'CPU'}")
        rospy.loginfo(f"订阅图像话题: {image_topic}")
        rospy.loginfo(f"发布边界框话题: {pub_topic}")
        rospy.loginfo(f"发布检测图像话题: {image_pub_topic}")
        rospy.loginfo(f"可视化显示: {self.visualize}")
        rospy.loginfo(f"增强过滤模式: {self.enhanced}")
        if self.enhanced:
            rospy.loginfo(f"过滤参数 filter: {filter}")
        rospy.loginfo("开始实时检测，按Ctrl+C退出...")

    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")
            return

        # >>> 新增：输出图像分辨率（仅当变化时） <<<
        current_resolution = (frame.shape[1], frame.shape[0])  # (width, height)
        if self.printed_resolution != current_resolution:
            rospy.loginfo(f"输入图像分辨率: {current_resolution[0]} x {current_resolution[1]}")
            self.printed_resolution = current_resolution

        # >>> 新增：计算并输出帧率 <<<
        current_time = rospy.Time.now()
        elapsed = (current_time - self.last_time).to_sec()
        if elapsed > 0.001:  # 避免除零
            self.fps = 1.0 / elapsed
        else:
            self.fps = 0.0
        self.last_time = current_time
        rospy.loginfo_throttle(5.0, f"当前检测帧率: {self.fps:.1f} FPS | 使用设备: {self.device}")

        # 使用YOLOv8进行预测
        results = self.model(frame, device=self.device)
        
        # 创建边界框消息
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = msg.header
        self.boundingBoxes.image_header = msg.header
        
        # 只发布最靠近中心的一个检测框（类型），并做类型稳定性判断
        img_height, img_width = frame.shape[:2]
        img_center = np.array([img_width / 2, img_height / 2])
        detected_type = None
        valid_boxes = []
        if len(results[0].boxes) > 0:
            for result in results[0].boxes:
                class_name = results[0].names[result.cls.item()]
                confidence = result.conf.item()
                # Enhanced模式下的过滤条件
                if self.enhanced:
                    if confidence < self.model.conf:
                        continue
                    length = abs(np.int64(result.xyxy[0][2].item() - result.xyxy[0][0].item()))
                    width = abs(np.int64(result.xyxy[0][3].item() - result.xyxy[0][1].item()))
                    if length >= filter * width or width >= filter * length:
                        continue
                # 计算检测框中心
                x_center = (result.xyxy[0][0].item() + result.xyxy[0][2].item()) / 2
                y_center = (result.xyxy[0][1].item() + result.xyxy[0][3].item()) / 2
                box_center = np.array([x_center, y_center])
                distance = np.linalg.norm(box_center - img_center)
                valid_boxes.append((result, distance, class_name, confidence, x_center, y_center))
            if valid_boxes:
                # 选出距离中心最近的检测框
                closest = min(valid_boxes, key=lambda x: x[1])
                result, _, class_name, confidence, x_center, y_center = closest
                detected_type = class_name
                # 类型稳定性判断
                if self.last_type is None:
                    self.last_type = detected_type
                    self.stable_count = 1
                    self.switch_pending = None
                    self.no_type_count = 0
                elif detected_type == self.last_type:
                    self.stable_count += 1
                    self.switch_pending = None
                    self.no_type_count = 0
                else:
                    if self.switch_pending == detected_type:
                        self.stable_count += 1
                    else:
                        self.switch_pending = detected_type
                        self.stable_count = 1
                    self.no_type_count = 0
                    # 只有稳定10帧才切换类型
                    if self.stable_count >= 10:
                        self.last_type = self.switch_pending
                        self.stable_count = 1
                        self.switch_pending = None
                # 输出逻辑
                if detected_type == self.last_type:
                    rospy.loginfo(f"检测到的类别: {class_name} ({confidence:.2f})")
                    boundingBox = BoundingBox()
                    boundingBox.xmin = int(x_center)
                    boundingBox.ymin = int(y_center)
                    boundingBox.xmax = 0
                    boundingBox.ymax = 0
                    boundingBox.Class = class_name
                    boundingBox.probability = confidence
                    self.boundingBoxes.bounding_boxes.append(boundingBox)
                    valid_boxes = [result]  # 只用于绘制
                else:
                    rospy.loginfo(f"检测到新类型 {detected_type}，等待稳定...（当前稳定帧数：{self.stable_count}）")
                    valid_boxes = []
            else:
                rospy.loginfo("未检测到符合条件的目标")
                self.no_type_count += 1
        else:
            rospy.loginfo("未检测到任何目标")
            valid_boxes = []
            self.no_type_count += 1
        
        # 发布边界框消息
        self.position_pub.publish(self.boundingBoxes)
        
        # 只绘制最靠近中心的检测框
        if valid_boxes:
            annotated_frame = frame.copy()
            box = valid_boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            class_name = results[0].names[box.cls.item()]
            confidence = box.conf.item()
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(annotated_frame, label, (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            annotated_frame = results[0].plot()
        
        # 在图像上叠加帧率信息（左上角）
        try:
            fps_text = None
            # 优先使用模型返回的推理时间计算 FPS（units: ms）
            if hasattr(results[0], 'speed') and isinstance(results[0].speed, dict) and results[0].speed.get('inference', 0) > 0:
                fps_val = 1000.0 / results[0].speed['inference']
                fps_text = f"FPS: {int(fps_val)}"
            else:
                fps_text = f"FPS: {self.fps:.1f}"

            # 使用更靠右和下的位置以免覆盖左上角信息
            pos = (20, 50)
            # 黑色粗描边以提高可读性
            cv2.putText(annotated_frame, fps_text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
            # 绿色文字
            cv2.putText(annotated_frame, fps_text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        except Exception:
            # 若图像类型不支持直接绘制（极少情况），忽略
            pass

        # 发布检测结果图像
        self.publish_detection_image(annotated_frame, msg.header)
        
        # 根据参数决定是否显示结果
        if self.visualize:
            # 调整图像大小为640x480
            resized_frame = cv2.resize(annotated_frame, (640, 480))
            cv2.imshow('YOLOv8实时检测', resized_frame)
            cv2.waitKey(1)
    
    def publish_detection_image(self, image, header):
        """发布检测结果图像"""
        try:
            # 将OpenCV图像转换为ROS图像消息
            detection_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            detection_msg.header = header
            self.image_pub.publish(detection_msg)
        except Exception as e:
            rospy.logerr(f"检测图像发布失败: {e}")

def main():
    try:
        detector = YOLODetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()