"""
YOLOv5 目标检测器
支持 ONNX 和 RDK BPU 格式
"""

import numpy as np
import cv2
import ctypes
import json
import os
from typing import List, Tuple, Optional, Dict, Any
import time


class hbSysMem_t(ctypes.Structure):
    _fields_ = [('phyAddr', ctypes.c_double), ('virAddr', ctypes.c_void_p), ('memSize', ctypes.c_int)]


class hbDNNQuantiShift_yt(ctypes.Structure):
    _fields_ = [('shiftLen', ctypes.c_int), ('shiftData', ctypes.c_char_p)]


class hbDNNQuantiScale_t(ctypes.Structure):
    _fields_ = [
        ('scaleLen', ctypes.c_int),
        ('scaleData', ctypes.POINTER(ctypes.c_float)),
        ('zeroPointLen', ctypes.c_int),
        ('zeroPointData', ctypes.c_char_p),
    ]


class hbDNNTensorShape_t(ctypes.Structure):
    _fields_ = [('dimensionSize', ctypes.c_int * 8), ('numDimensions', ctypes.c_int)]


class hbDNNTensorProperties_t(ctypes.Structure):
    _fields_ = [
        ('validShape', hbDNNTensorShape_t),
        ('alignedShape', hbDNNTensorShape_t),
        ('tensorLayout', ctypes.c_int),
        ('tensorType', ctypes.c_int),
        ('shift', hbDNNQuantiShift_yt),
        ('scale', hbDNNQuantiScale_t),
        ('quantiType', ctypes.c_int),
        ('quantizeAxis', ctypes.c_int),
        ('alignedByteSize', ctypes.c_int),
        ('stride', ctypes.c_int * 8),
    ]


class hbDNNTensor_t(ctypes.Structure):
    _fields_ = [('sysMem', hbSysMem_t * 4), ('properties', hbDNNTensorProperties_t)]


class Yolov5PostProcessInfo_t(ctypes.Structure):
    _fields_ = [
        ('height', ctypes.c_int),
        ('width', ctypes.c_int),
        ('ori_height', ctypes.c_int),
        ('ori_width', ctypes.c_int),
        ('score_threshold', ctypes.c_float),
        ('nms_threshold', ctypes.c_float),
        ('nms_top_k', ctypes.c_int),
        ('is_pad_resize', ctypes.c_int),
    ]


class YOLODetector:
    """
    YOLOv5 目标检测器
    
    支持:
    - ONNX Runtime 推理
    - RDK BPU 推理 (地平线)
    - OpenCV DNN 后备
    """
    
    # COCO 数据集类别名称
    COCO_CLASSES = [
        'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
        'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
        'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
        'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
        'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
        'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
        'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
        'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
        'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
        'toothbrush'
    ]
    
    def __init__(
        self,
        model_path: str,
        input_size: Tuple[int, int] = (640, 640),
        conf_threshold: float = 0.45,
        iou_threshold: float = 0.45,
        backend: str = "auto",  # "onnx", "bpu", "opencv", "auto"
        device: str = "cpu",
    ):
        """
        初始化 YOLO 检测器
        
        Args:
            model_path: 模型文件路径 (.onnx 或 .bin)
            input_size: 输入尺寸 (H, W)
            conf_threshold: 置信度阈值
            iou_threshold: NMS IoU 阈值
            backend: 推理后端 ("onnx", "bpu", "opencv", "auto")
            device: 计算设备
        """
        self.model_path = model_path
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.backend = backend
        self.device = device
        
        # 模型实例
        self.session = None
        self.input_name = None
        self.output_names = None
        self._hobot_model = None
        self._hobot_postprocess = None
        self._hobot_get_result = None
        
        # 类别名称
        self.class_names = self.COCO_CLASSES
        self.num_classes = len(self.class_names)
        
        # 颜色调色板
        self.colors = self._generate_colors()
        
        # 初始化模型
        self._load_model()
    
    def _generate_colors(self) -> np.ndarray:
        """生成颜色调色板"""
        np.random.seed(42)
        colors = np.random.randint(0, 255, size=(self.num_classes, 3), dtype=np.uint8)
        return colors
    
    def _load_model(self):
        """加载模型"""
        if self.backend == "auto":
            # 自动检测后端
            if self.model_path.endswith(".onnx"):
                self.backend = "onnx"
            elif self.model_path.endswith(".bin"):
                self.backend = "bpu"
            else:
                self.backend = "opencv"
        
        if self.backend == "onnx":
            self._load_onnx()
        elif self.backend == "bpu":
            self._load_bpu()
        elif self.backend == "opencv":
            self._load_opencv()
        else:
            raise ValueError(f"不支持的后端: {self.backend}")
    
    def _load_onnx(self):
        """加载 ONNX 模型"""
        try:
            import onnxruntime as ort
            
            # 创建推理会话
            providers = ['CPUExecutionProvider']
            if self.device == "cuda":
                providers.insert(0, 'CUDAExecutionProvider')
            
            self.session = ort.InferenceSession(
                self.model_path,
                providers=providers
            )
            
            # 获取输入输出名称
            self.input_name = self.session.get_inputs()[0].name
            self.output_names = [output.name for output in self.session.get_outputs()]
            
            # 获取输入尺寸
            input_shape = self.session.get_inputs()[0].shape
            if len(input_shape) == 4:
                self.input_size = (input_shape[2], input_shape[3])
            
            print(f"✅ ONNX 模型加载成功: {self.model_path}")
            print(f"   输入: {self.input_name} {input_shape}")
            print(f"   输出: {self.output_names}")
            
        except ImportError:
            print("❌ 未安装 onnxruntime，请运行: pip3 install onnxruntime")
            raise
        except Exception as e:
            print(f"❌ ONNX 模型加载失败: {e}")
            raise
    
    def _load_bpu(self):
        """
        加载 RDK BPU 模型
        
        RDK (地平线) BPU 推理方式：
        1. horizon_tc Runtime (推荐) - 地平线工具链运行时
        2. hbDNN (底层 API) - 地平线 DNN API
        3. OpenCV DNN with RDK backend - OpenCV 4.x+ 支持
        """
        # 方式1: RDK X5 常用 hobot_dnn Runtime
        try:
            from hobot_dnn import pyeasy_dnn as dnn
            models = dnn.load(self.model_path)
            self._hobot_model = models[0] if isinstance(models, list) else models
            self._load_hobot_postprocess()
            self.input_size = self._get_hobot_model_hw(self._hobot_model)
            self.input_name = self._hobot_model.inputs[0].name
            self.output_names = [output.name for output in self._hobot_model.outputs]
            self.backend = 'hobot_dnn'
            print(f"✅ RDK BPU 模型加载成功 (hobot_dnn): {self.model_path}")
            print(f"   输入尺寸: {self.input_size}, 输出: {self.output_names}")
            return
        except ImportError:
            pass
        except Exception as e:
            print(f"⚠️  hobot_dnn 加载失败: {e}")

        # 方式2: 尝试 horizon_tc Runtime
        try:
            from horizon_tc import Runtime
            self._bpu_runtime = Runtime(self.model_path)
            self._bpu_input = self._bpu_runtime.get_input(0)
            self._bpu_output = self._bpu_runtime.get_output(0)
            self.input_name = 'input'
            self.output_names = ['output']
            print(f"✅ RDK BPU 模型加载成功 (horizon_tc): {self.model_path}")
            return
        except ImportError:
            pass
        except Exception as e:
            print(f"⚠️  horizon_tc 加载失败: {e}")

        # 方式3: 尝试 hbDNN
        try:
            import hbDNN
            self._bpu_model = hbDNN.load(self.model_path)
            self.input_name = 'input'
            self.output_names = ['output']
            print(f"✅ RDK BPU 模型加载成功 (hbDNN): {self.model_path}")
            return
        except ImportError:
            pass
        except Exception as e:
            print(f"⚠️  hbDNN 加载失败: {e}")

        # 方式4: 尝试 OpenCV DNN (支持 RDK 后端)
        try:
            self.net = cv2.dnn.readNet(self.model_path)
            # 尝试设置 RDK BPU 后端
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_HALIDE)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                print(f"✅ RDK BPU 模型加载成功 (OpenCV DNN): {self.model_path}")
                self.backend = "opencv_bpu"
                return
            except Exception:
                pass
        except Exception:
            pass

        # 回退: 尝试查找同名 .onnx 文件
        print("⚠️  RDK BPU SDK 未安装，尝试回退到 ONNX 模式...")
        onnx_path = self.model_path.replace(".bin", ".onnx")
        import os
        if os.path.exists(onnx_path):
            self.model_path = onnx_path
            self.backend = "onnx"
            self._load_onnx()
        else:
            print("⚠️  未找到 ONNX 模型，使用 OpenCV DNN 后备方案")
            self.backend = "opencv"
            self._load_opencv()

    def _load_hobot_postprocess(self):
        lib_path = '/usr/lib/libpostprocess.so'
        if not os.path.exists(lib_path):
            raise FileNotFoundError(f'未找到 RDK YOLO 后处理库: {lib_path}')
        lib = ctypes.CDLL(lib_path)
        self._hobot_postprocess = lib.Yolov5doProcess
        self._hobot_get_result = lib.Yolov5PostProcess
        self._hobot_get_result.argtypes = [ctypes.POINTER(Yolov5PostProcessInfo_t)]
        self._hobot_get_result.restype = ctypes.c_char_p

    @staticmethod
    def _get_hobot_model_hw(model):
        props = model.inputs[0].properties
        if props.layout == 'NCHW':
            return int(props.shape[2]), int(props.shape[3])
        return int(props.shape[1]), int(props.shape[2])
    
    def _load_opencv(self):
        """使用 OpenCV DNN 加载模型"""
        try:
            self.net = cv2.dnn.readNetFromONNX(self.model_path)
            
            # 设置后端
            if self.device == "cuda":
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            else:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            
            print(f"✅ OpenCV DNN 模型加载成功: {self.model_path}")
            
        except Exception as e:
            print(f"❌ OpenCV DNN 模型加载失败: {e}")
            raise
    
    def detect(
        self,
        image: np.ndarray,
        classes: Optional[List[int]] = None,
    ) -> List[Dict[str, Any]]:
        """
        执行目标检测
        
        Args:
            image: 输入图像 (H, W, 3), BGR 格式
            classes: 过滤的类别 ID 列表
            
        Returns:
            detections: 检测结果列表
                - bbox: [x1, y1, x2, y2]
                - score: 置信度
                - class_id: 类别 ID
                - class_name: 类别名称
        """
        if self.backend == 'hobot_dnn':
            detections = self._detect_hobot_dnn(image)
            if classes is not None:
                detections = [d for d in detections if d['class_id'] in classes]
            return detections

        # 预处理
        input_tensor, ratio, pad = self._preprocess(image)
        
        # 推理
        if self.backend == "onnx":
            outputs = self._infer_onnx(input_tensor)
        elif self.backend == "bpu":
            outputs = self._infer_bpu(input_tensor)
        elif self.backend == "opencv" or self.backend == "opencv_bpu":
            outputs = self._infer_opencv(input_tensor)
        else:
            outputs = self._infer_onnx(input_tensor)
        
        # 后处理
        detections = self._postprocess(outputs, ratio, pad, image.shape[:2])
        
        # 过滤类别
        if classes is not None:
            detections = [d for d in detections if d['class_id'] in classes]
        
        return detections

    def _detect_hobot_dnn(self, image: np.ndarray) -> List[Dict[str, Any]]:
        input_tensor = self._preprocess_hobot_nv12(image)
        self._hobot_model.forward(input_tensor)
        return self._postprocess_hobot(image.shape[:2])

    def _preprocess_hobot_nv12(self, image: np.ndarray) -> np.ndarray:
        target_h, target_w = self.input_size
        resized = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_AREA)
        return self._bgr_to_nv12(resized)

    @staticmethod
    def _bgr_to_nv12(image: np.ndarray) -> np.ndarray:
        height, width = image.shape[:2]
        area = height * width
        yuv420p = cv2.cvtColor(image, cv2.COLOR_BGR2YUV_I420).reshape((area * 3 // 2,))
        y = yuv420p[:area]
        uv_planar = yuv420p[area:].reshape((2, area // 4))
        uv_packed = uv_planar.transpose((1, 0)).reshape((area // 2,))
        nv12 = np.empty_like(yuv420p)
        nv12[:area] = y
        nv12[area:] = uv_packed
        return nv12

    def _postprocess_hobot(self, image_shape: Tuple[int, int]) -> List[Dict[str, Any]]:
        image_h, image_w = image_shape
        model_h, model_w = self.input_size

        pp_info = Yolov5PostProcessInfo_t()
        pp_info.height = model_h
        pp_info.width = model_w
        pp_info.ori_height = int(image_h)
        pp_info.ori_width = int(image_w)
        pp_info.score_threshold = ctypes.c_float(self.conf_threshold)
        pp_info.nms_threshold = ctypes.c_float(self.iou_threshold)
        pp_info.nms_top_k = 100
        pp_info.is_pad_resize = 0

        output_tensors = (hbDNNTensor_t * len(self._hobot_model.outputs))()
        for index, output in enumerate(self._hobot_model.outputs):
            props = output.properties
            tensor = output_tensors[index]
            tensor.properties.tensorLayout = 2 if props.layout == 'NCHW' else 0

            if len(props.scale_data) == 0:
                tensor.properties.quantiType = 0
                tensor.sysMem[0].virAddr = ctypes.cast(
                    output.buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_float)),
                    ctypes.c_void_p,
                )
            else:
                tensor.properties.quantiType = 2
                tensor.properties.scale.scaleData = props.scale_data.ctypes.data_as(
                    ctypes.POINTER(ctypes.c_float)
                )
                tensor.sysMem[0].virAddr = ctypes.cast(
                    output.buffer.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
                    ctypes.c_void_p,
                )

            for dim_index, dim in enumerate(props.shape):
                tensor.properties.validShape.dimensionSize[dim_index] = int(dim)
            tensor.properties.validShape.numDimensions = len(props.shape)
            self._hobot_postprocess(output_tensors[index], ctypes.pointer(pp_info), index)

        result = self._hobot_get_result(ctypes.pointer(pp_info))
        if not result:
            return []
        result_text = result.decode('utf-8')
        json_start = result_text.find('[')
        if json_start < 0:
            return []

        detections = []
        for item in json.loads(result_text[json_start:]):
            class_id = int(item.get('id', 0))
            bbox = item.get('bbox', [0.0, 0.0, 0.0, 0.0])
            detections.append({
                'bbox': [float(v) for v in bbox[:4]],
                'score': float(item.get('score', 0.0)),
                'class_id': class_id,
                'class_name': self.class_names[class_id] if class_id < len(self.class_names) else f'class_{class_id}',
            })
        return detections
    
    def _preprocess(self, image: np.ndarray) -> Tuple[np.ndarray, float, Tuple[int, int]]:
        """
        图像预处理
        
        Args:
            image: 输入图像
            
        Returns:
            input_tensor: 预处理后的张量
            ratio: 缩放比例
            pad: 填充 (top, left)
        """
        h, w = image.shape[:2]
        target_h, target_w = self.input_size
        
        # 计算缩放比例
        ratio = min(target_w / w, target_h / h)
        new_w = int(w * ratio)
        new_h = int(h * ratio)
        
        # 缩放图像
        image_resized = cv2.resize(image, (new_w, new_h))
        
        # 创建画布并填充
        canvas = np.full((target_h, target_w, 3), 114, dtype=np.uint8)
        
        # 计算填充位置（居中）
        pad_top = (target_h - new_h) // 2
        pad_left = (target_w - new_w) // 2
        
        canvas[pad_top:pad_top + new_h, pad_left:pad_left + new_w] = image_resized
        
        # 转换为 float32 并归一化
        input_tensor = canvas.astype(np.float32) / 255.0
        
        # 转换为 NCHW 格式
        input_tensor = np.transpose(input_tensor, (2, 0, 1))
        input_tensor = np.expand_dims(input_tensor, axis=0)
        
        # 检查模型期望的数据类型
        if hasattr(self, 'session') and self.session is not None:
            expected_type = self.session.get_inputs()[0].type
            if 'float16' in expected_type:
                input_tensor = input_tensor.astype(np.float16)
        
        # 确保内存连续
        input_tensor = np.ascontiguousarray(input_tensor)
        
        return input_tensor, ratio, (pad_top, pad_left)
    
    def _infer_onnx(self, input_tensor: np.ndarray) -> np.ndarray:
        """ONNX 推理"""
        outputs = self.session.run(
            self.output_names,
            {self.input_name: input_tensor}
        )
        return outputs[0]
    
    def _infer_bpu(self, input_tensor: np.ndarray) -> np.ndarray:
        """
        RDK BPU 推理
        
        支持三种 BPU 运行时：
        1. horizon_tc Runtime
        2. hbDNN
        3. OpenCV DNN with BPU backend
        """
        # 方式1: horizon_tc Runtime
        if hasattr(self, '_bpu_runtime'):
            try:
                import horizon_tc
                # 设置输入数据
                input_data = input_tensor.astype(np.float32)
                self._bpu_input.set_data(input_data)
                # 执行推理
                self._bpu_runtime.run()
                # 获取输出
                output_data = self._bpu_output.get_data()
                return output_data
            except Exception as e:
                print(f"⚠️  horizon_tc 推理失败: {e}")
        
        # 方式2: hbDNN
        if hasattr(self, '_bpu_model'):
            try:
                import hbDNN
                input_data = input_tensor.astype(np.float32)
                output = hbDNN.infer(self._bpu_model, input_data)
                return output
            except Exception as e:
                print(f"⚠️  hbDNN 推理失败: {e}")
        
        # 方式3: OpenCV DNN with BPU backend
        if hasattr(self, 'net') and self.backend == "opencv_bpu":
            try:
                self.net.setInput(input_tensor)
                outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
                return outputs[0]
            except Exception as e:
                print(f"⚠️  OpenCV BPU 推理失败: {e}")
        
        # 回退到 ONNX
        if self.session is not None:
            print("⚠️  BPU 推理失败，回退到 ONNX")
            return self._infer_onnx(input_tensor)
        
        raise RuntimeError("BPU 推理失败且无可用的回退后端")
    
    def _infer_opencv(self, input_tensor: np.ndarray) -> np.ndarray:
        """OpenCV DNN 推理"""
        self.net.setInput(input_tensor)
        outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
        return outputs[0]
    
    def _postprocess(
        self,
        outputs: np.ndarray,
        ratio: float,
        pad: Tuple[int, int],
        image_shape: Tuple[int, int],
    ) -> List[Dict[str, Any]]:
        """
        后处理：解析模型输出，执行 NMS
        
        Args:
            outputs: 模型输出
            ratio: 缩放比例
            pad: 填充 (top, left)
            image_shape: 原始图像尺寸 (H, W)
            
        Returns:
            detections: 检测结果列表
        """
        # YOLOv5 输出格式: [batch, num_detections, 5 + num_classes]
        # 其中 5 = x_center, y_center, width, height, confidence
        
        if len(outputs.shape) == 3:
            outputs = outputs[0]  # 移除 batch 维度
        
        # 提取边界框和置信度
        boxes = outputs[:, :4]  # x_center, y_center, width, height
        scores = outputs[:, 4]  # objectness score
        class_scores = outputs[:, 5:]  # class scores
        
        # 计算最终置信度
        max_class_scores = np.max(class_scores, axis=1)
        final_scores = scores * max_class_scores
        
        # 过滤低置信度
        mask = final_scores > self.conf_threshold
        boxes = boxes[mask]
        final_scores = final_scores[mask]
        class_scores = class_scores[mask]
        
        if len(boxes) == 0:
            return []
        
        # 转换边界框格式: xywh -> xyxy
        boxes_xyxy = np.zeros_like(boxes)
        boxes_xyxy[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # x1
        boxes_xyxy[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # y1
        boxes_xyxy[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # x2
        boxes_xyxy[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # y2
        
        # 反向变换到原始图像坐标
        pad_top, pad_left = pad
        boxes_xyxy[:, [0, 2]] = (boxes_xyxy[:, [0, 2]] - pad_left) / ratio
        boxes_xyxy[:, [1, 3]] = (boxes_xyxy[:, [1, 3]] - pad_top) / ratio
        
        # 裁剪到图像范围
        h, w = image_shape
        boxes_xyxy[:, [0, 2]] = np.clip(boxes_xyxy[:, [0, 2]], 0, w)
        boxes_xyxy[:, [1, 3]] = np.clip(boxes_xyxy[:, [1, 3]], 0, h)
        
        # 获取类别 ID
        class_ids = np.argmax(class_scores, axis=1)
        
        # OpenCV NMSBoxes expects [x, y, width, height], not [x1, y1, x2, y2].
        nms_boxes = boxes_xyxy.copy()
        nms_boxes[:, 2] = nms_boxes[:, 2] - nms_boxes[:, 0]
        nms_boxes[:, 3] = nms_boxes[:, 3] - nms_boxes[:, 1]

        # 执行 NMS
        indices = cv2.dnn.NMSBoxes(
            nms_boxes.tolist(),
            final_scores.tolist(),
            self.conf_threshold,
            self.iou_threshold
        )
        
        # 构建结果
        detections = []
        
        if len(indices) > 0:
            for idx in indices.flatten():
                detection = {
                    'bbox': boxes_xyxy[idx].tolist(),
                    'score': float(final_scores[idx]),
                    'class_id': int(class_ids[idx]),
                    'class_name': self.class_names[int(class_ids[idx])],
                }
                detections.append(detection)
        
        return detections
    
    def draw_detections(
        self,
        image: np.ndarray,
        detections: List[Dict[str, Any]],
        show_labels: bool = True,
        show_scores: bool = True,
    ) -> np.ndarray:
        """
        在图像上绘制检测结果
        
        Args:
            image: 输入图像
            detections: 检测结果列表
            show_labels: 是否显示标签
            show_scores: 是否显示置信度
            
        Returns:
            image: 绘制后的图像
        """
        image_copy = image.copy()
        
        for det in detections:
            bbox = det['bbox']
            score = det['score']
            class_id = det['class_id']
            class_name = det['class_name']
            
            x1, y1, x2, y2 = [int(v) for v in bbox]
            
            # 获取颜色
            color = tuple(map(int, self.colors[class_id]))
            
            # 绘制边界框
            cv2.rectangle(image_copy, (x1, y1), (x2, y2), color, 2)
            
            # 构建标签
            label_parts = []
            if show_labels:
                label_parts.append(class_name)
            if show_scores:
                label_parts.append(f"{score:.2f}")
            
            if label_parts:
                label = " ".join(label_parts)
                
                # 计算标签大小
                (label_w, label_h), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )
                
                # 绘制标签背景
                cv2.rectangle(
                    image_copy,
                    (x1, y1 - label_h - baseline - 5),
                    (x1 + label_w, y1),
                    color,
                    -1
                )
                
                # 绘制标签文字
                cv2.putText(
                    image_copy,
                    label,
                    (x1, y1 - baseline - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA
                )
        
        return image_copy
    
    def __del__(self):
        """析构函数"""
        if self.session is not None:
            del self.session


class YOLOv5Detector(YOLODetector):
    """
    YOLOv5 检测器
    
    这是 YOLODetector 的别名，用于向后兼容
    """
    pass
