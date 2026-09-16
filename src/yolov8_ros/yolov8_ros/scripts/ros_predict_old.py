#!/usr/bin/env python
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
        weight_path = rospy.get_param('~weight_path', '/home/phoenixtech/abot_ws/src/yolov8_ros/yolov8_ros/weights/gjs.pt')
        
        # 获取置信度参数
        conf = rospy.get_param('~conf', 0.5)
        
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
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        
        # 发布边界框消息
        self.position_pub = rospy.Publisher(pub_topic, BoundingBoxes, queue_size=1)
        
        # 发布检测结果图像
        self.image_pub = rospy.Publisher(image_pub_topic, Image, queue_size=1)
        
        print(f"加载模型: {weight_path}")
        print(f"使用设备: {self.device}")
        print(f"CUDA可用: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"CUDA设备名称: {torch.cuda.get_device_name(0)}")
        print(f"订阅图像话题: {image_topic}")
        print(f"发布边界框话题: {pub_topic}")
        print(f"发布检测图像话题: {image_pub_topic}")
        print(f"可视化显示: {self.visualize}")
        print(f"增强过滤模式: {self.enhanced}")
        if self.enhanced:
            print(f"过滤参数 filter: {filter}")
        print("开始实时检测，按Ctrl+C退出...")
    
    def image_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")
            return
        
        # 使用YOLOv8进行预测（使用CUDA加速）
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
                
                # tank 的特殊处理逻辑
                if class_name == 'tank' and confidence > 0.5:
                    # 对于 tank，直接添加到 valid_boxes，不执行其他过滤
                    x_center = (result.xyxy[0][0].item() + result.xyxy[0][2].item()) / 2
                    y_center = (result.xyxy[0][1].item() + result.xyxy[0][3].item()) / 2
                    box_center = np.array([x_center, y_center])
                    distance = np.linalg.norm(box_center - img_center)
                    valid_boxes.append((result, distance, class_name, confidence, x_center, y_center))
                    continue
                    
                # 其他目标的Enhanced模式下的过滤条件
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
                # tank 的特殊处理
                if class_name == 'tank':
                    print(f"检测到特殊目标 tank: {confidence:.2f}")
                    boundingBox = BoundingBox()
                    boundingBox.xmin = int(x_center)
                    boundingBox.ymin = int(y_center)
                    boundingBox.xmax = 0
                    boundingBox.ymax = 0
                    boundingBox.Class = class_name
                    boundingBox.probability = confidence
                    self.boundingBoxes.bounding_boxes.append(boundingBox)
                    valid_boxes = [result]  # 只用于绘制
                # 其他目标的类型稳定性处理
                elif detected_type == self.last_type:
                    print(f"检测到的类别: {class_name}({confidence:.2f})")
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
                    print(f"检测到新类型 {detected_type}，等待稳定...（当前稳定帧数：{self.stable_count}）")
                    valid_boxes = []
            else:
                print("未检测到符合条件的目标")
                self.no_type_count += 1
                # 若连续10帧未检测到任何类型，允许切换类型
                if self.no_type_count >= 10 and self.switch_pending is not None:
                    self.last_type = self.switch_pending
                    self.stable_count = 1
                    self.switch_pending = None
        else:
            print("未检测到任何目标")
            valid_boxes = []
            self.no_type_count += 1
            # 若连续10帧未检测到任何类型，允许切换类型
            if self.no_type_count >= 10 and self.switch_pending is not None:
                self.last_type = self.switch_pending
                self.stable_count = 1
                self.switch_pending = None
        
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
