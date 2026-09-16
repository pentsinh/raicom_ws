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
        weight_path = rospy.get_param('~weight_path', '/home/phoenixtech/abot_ws/src/yolov8_ros/yolov8_ros/weights/gjs.pt')
        
        # 获取置信度参数
        conf = rospy.get_param('~conf', 0.5)

        # 降低推理开销的超参数
        self.imgsz = int(rospy.get_param('~imgsz', 512))
        self.max_det = int(rospy.get_param('~max_det', 10))
        
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
        self.conf = conf
        self.half = bool(rospy.get_param('~half', self.device == 'cuda'))
        if self.device == 'cpu':
            self.half = False
        
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
        self.image_sub = rospy.Subscriber(
            image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
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
        print(f"推理尺寸: {self.imgsz}")
        print(f"最大检测数: {self.max_det}")
        print(f"半精度推理: {self.half}")
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
        with torch.no_grad():
            results = self.model.predict(
                source=frame,
                device=self.device,
                imgsz=self.imgsz,
                conf=self.conf,
                half=self.half,
                max_det=self.max_det,
                verbose=False,
            )
        
        # 创建边界框消息
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = msg.header
        self.boundingBoxes.image_header = msg.header
        
        # 只发布最靠近中心的一个检测框（类型），并做类型稳定性判断
        img_height, img_width = frame.shape[:2]
        img_center = np.array([img_width / 2, img_height / 2])
        detected_type = None
        valid_boxes = []
        random_boxes = []  # 专门存储random类型的框
        if len(results[0].boxes) > 0:
            for result in results[0].boxes:
                class_name = results[0].names[result.cls.item()]
                confidence = result.conf.item()
                
                # random 的特殊处理逻辑 - 置信度>0.6就直接发送
                if class_name == 'random' and confidence > 0.75:
                    x_center = (result.xyxy[0][0].item() + result.xyxy[0][2].item()) / 2
                    y_center = (result.xyxy[0][1].item() + result.xyxy[0][3].item()) / 2
                    # 直接添加到random_boxes，不参与中心点选择
                    random_boxes.append((result, class_name, confidence, x_center, y_center))
                    print(f"检测到特殊目标 random: {confidence:.2f}")
                    # 立即发送random类型的目标
                    boundingBox = BoundingBox()
                    boundingBox.xmin = int(x_center)
                    boundingBox.ymin = int(y_center)
                    boundingBox.xmax = 0
                    boundingBox.ymax = 0
                    boundingBox.Class = class_name
                    boundingBox.probability = confidence
                    self.boundingBoxes.bounding_boxes.append(boundingBox)
                    continue
                    
                # 其他目标的Enhanced模式下的过滤条件
                if self.enhanced:
                    if confidence < self.conf:
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

            # ========== 新增：检测框合并逻辑（避免破碎） ==========
            if len(valid_boxes) > 1:
                merged_boxes = []
                used = [False] * len(valid_boxes)
                merge_distance_threshold = 50.0  # 中心点距离小于该值则合并

                for i in range(len(valid_boxes)):
                    if used[i]:
                        continue
                    cluster = [valid_boxes[i]]
                    used[i] = True
                    center_i = np.array([valid_boxes[i][4], valid_boxes[i][5]])  # (x_center, y_center)

                    for j in range(i + 1, len(valid_boxes)):
                        if used[j]:
                            continue
                        center_j = np.array([valid_boxes[j][4], valid_boxes[j][5]])
                        if np.linalg.norm(center_i - center_j) < merge_distance_threshold:
                            cluster.append(valid_boxes[j])
                            used[j] = True

                    if len(cluster) > 1:
                        # 合并坐标
                        orig_results = [item[0] for item in cluster]
                        x1s = [r.xyxy[0][0].item() for r in orig_results]
                        y1s = [r.xyxy[0][1].item() for r in orig_results]
                        x2s = [r.xyxy[0][2].item() for r in orig_results]
                        y2s = [r.xyxy[0][3].item() for r in orig_results]
                        merged_x1, merged_y1 = min(x1s), min(y1s)
                        merged_x2, merged_y2 = max(x2s), max(y2s)

                        # 类别和置信度：取集群中最高者
                        best_item = max(cluster, key=lambda x: x[3])
                        best_class = best_item[2]
                        best_conf = best_item[3]

                        # 构造合并后的虚拟 result 对象
                        from types import SimpleNamespace
                        merged_result = SimpleNamespace()
                        merged_result.xyxy = torch.tensor([[[merged_x1, merged_y1, merged_x2, merged_y2]]])
                        # 获取类别索引
                        cls_id = list(results[0].names.keys())[list(results[0].names.values()).index(best_class)]
                        merged_result.cls = torch.tensor([cls_id])
                        merged_result.conf = torch.tensor([best_conf])

                        # 重新计算中心
                        merged_x_center = (merged_x1 + merged_x2) / 2
                        merged_y_center = (merged_y1 + merged_y2) / 2
                        merged_distance = np.linalg.norm(np.array([merged_x_center, merged_y_center]) - img_center)

                        merged_boxes.append((
                            merged_result,
                            merged_distance,
                            best_class,
                            best_conf,
                            merged_x_center,
                            merged_y_center
                        ))
                    else:
                        # 无合并，保留原框
                        merged_boxes.append(cluster[0])

                valid_boxes = merged_boxes
            # ========== 合并逻辑结束 ==========

            # 处理非random类型的目标 - 只选择最靠近中心的一个
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
                
                # 其他目标的类型稳定性处理
                if detected_type == self.last_type:
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
        
        # 绘制检测框 - 包括random类型和最靠近中心的非random类型
        annotated_frame = frame.copy()
        
        # 绘制random类型的框
        for result, class_name, confidence, x_center, y_center in random_boxes:
            x1, y1, x2, y2 = result.xyxy[0].cpu().numpy()
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(annotated_frame, label, (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 绘制最靠近中心的非random类型框
        if valid_boxes:
            box = valid_boxes[0]
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            class_name = results[0].names[box.cls.item()]
            confidence = box.conf.item()
            cv2.rectangle(annotated_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(annotated_frame, label, (int(x1), int(y1) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 发布检测结果图像
        self.publish_detection_image(annotated_frame, msg.header)
        
        # 根据参数决定是否显示结果
        if self.visualize:
            # 调整图像大小为640x480
            resized_frame = cv2.resize(annotated_frame, (960,540))
            cv2.imshow('YOLOv8实时检测', resized_frame)
            cv2.waitKey(1)
    
    def publish_detection_image(self, image, header):
        """发布检测结果图像
        手动构造 sensor_msgs/Image：cv_bridge 的 cv2_to_imgmsg 在本机环境会因
        KeyError: 16（CV_8UC3 转换表缺失）抛异常，这里绕开它。
        """
        try:
            detection_msg = Image()
            detection_msg.header = header
            detection_msg.height, detection_msg.width = image.shape[:2]
            detection_msg.encoding = "bgr8"
            detection_msg.is_bigendian = 0
            detection_msg.step = image.shape[1] * image.shape[2]
            detection_msg.data = image.tobytes()
            self.image_pub.publish(detection_msg)
        except Exception as e:
            rospy.logerr(f"检测图像发布失败: {type(e).__name__}: {e} "
                         f"(image type={type(image).__name__}, "
                         f"shape={getattr(image, 'shape', None)}, "
                         f"dtype={getattr(image, 'dtype', None)})")

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