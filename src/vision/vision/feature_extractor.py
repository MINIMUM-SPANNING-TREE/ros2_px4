"""
DeepSORT 特征提取器
支持 ONNX 和 RDK BPU 格式的 ReID 模型
"""

import numpy as np
import cv2
from typing import Optional, List, Tuple


class FeatureExtractor:
    """
    外观特征提取器
    
    支持:
    - ONNX Runtime 推理
    - RDK BPU 推理 (地平线)
    - 简单颜色直方图 (后备)
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        input_size: Tuple[int, int] = (256, 128),  # ReID 标准输入 (H, W)
        feature_dim: int = 512,
        backend: str = "auto",  # "onnx", "bpu", "simple", "auto"
        device: str = "cpu",
    ):
        """
        初始化特征提取器
        
        Args:
            model_path: ReID 模型文件路径 (.onnx 或 .bin)
            input_size: 输入图像尺寸 (H, W)
            feature_dim: 特征维度
            backend: 推理后端
            device: 计算设备
        """
        self.model_path = model_path
        self.input_size = input_size
        self.feature_dim = feature_dim
        self.backend = backend
        self.device = device
        
        # 模型实例
        self.session = None
        self.input_name = None
        self.output_name = None
        
        # 初始化
        self._load_model()
    
    def _load_model(self):
        """加载模型"""
        # 自动检测后端
        if self.backend == "auto":
            if self.model_path and self.model_path.endswith(".onnx"):
                self.backend = "onnx"
            elif self.model_path and self.model_path.endswith(".bin"):
                self.backend = "bpu"
            else:
                self.backend = "simple"
        
        # 加载模型
        if self.backend == "onnx":
            self._load_onnx()
        elif self.backend == "bpu":
            self._load_bpu()
        else:
            self.backend = "simple"
            print("✅ 使用简单特征提取器 (颜色直方图)")
    
    def _load_onnx(self):
        """加载 ONNX ReID 模型"""
        if not self.model_path:
            print("⚠️  未指定 ReID 模型路径，使用简单特征")
            self.backend = "simple"
            return
        
        try:
            import onnxruntime as ort
            
            providers = ['CPUExecutionProvider']
            if self.device == "cuda":
                providers.insert(0, 'CUDAExecutionProvider')
            
            self.session = ort.InferenceSession(
                self.model_path,
                providers=providers,
            )
            
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            
            # 获取输入尺寸
            input_shape = self.session.get_inputs()[0].shape
            if len(input_shape) == 4:
                self.input_size = (input_shape[2], input_shape[3])
            
            # 获取输出维度
            output_shape = self.session.get_outputs()[0].shape
            if len(output_shape) >= 2:
                self.feature_dim = output_shape[-1]
            
            print(f"✅ ReID ONNX 模型已加载: {self.model_path}")
            print(f"   输入: {self.input_name} {input_shape}")
            print(f"   输出: {self.output_name} {output_shape}")
            print(f"   特征维度: {self.feature_dim}")
            
        except ImportError:
            print("❌ 未安装 onnxruntime，使用简单特征")
            self.backend = "simple"
        except Exception as e:
            print(f"❌ ONNX 模型加载失败: {e}")
            self.backend = "simple"
    
    def _load_bpu(self):
        """
        加载 RDK BPU ReID 模型
        
        支持三种 BPU 运行时：
        1. horizon_tc Runtime (推荐)
        2. hbDNN
        3. OpenCV DNN with BPU backend
        """
        # 方式1: 尝试 horizon_tc Runtime
        try:
            from horizon_tc import Runtime
            self._bpu_runtime = Runtime(self.model_path)
            self._bpu_input = self._bpu_runtime.get_input(0)
            self._bpu_output = self._bpu_runtime.get_output(0)
            self.input_name = 'input'
            self.output_name = 'output'
            print(f"✅ ReID BPU 模型已加载 (horizon_tc): {self.model_path}")
            return
        except ImportError:
            pass
        except Exception as e:
            print(f"⚠️  horizon_tc 加载失败: {e}")

        # 方式2: 尝试 hbDNN
        try:
            import hbDNN
            self._bpu_model = hbDNN.load(self.model_path)
            self.input_name = 'input'
            self.output_name = 'output'
            print(f"✅ ReID BPU 模型已加载 (hbDNN): {self.model_path}")
            return
        except ImportError:
            pass
        except Exception as e:
            print(f"⚠️  hbDNN 加载失败: {e}")

        # 方式3: 尝试 OpenCV DNN
        try:
            self.net = cv2.dnn.readNet(self.model_path)
            try:
                self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_HALIDE)
                self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
                print(f"✅ ReID BPU 模型已加载 (OpenCV DNN): {self.model_path}")
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
            print("⚠️  未找到 ONNX 模型，使用简单特征提取")
            self.backend = "simple"
    
    def extract(
        self,
        image: np.ndarray,
        bboxes: np.ndarray,
    ) -> np.ndarray:
        """
        从图像中提取目标的外观特征
        
        Args:
            image: 输入图像 (H, W, 3), BGR 格式
            bboxes: 边界框数组 (N, 4) [x1, y1, x2, y2]
            
        Returns:
            features: 特征数组 (N, D)
        """
        if len(bboxes) == 0:
            return np.array([]).reshape(0, self.feature_dim)
        
        if self.backend == "onnx" and self.session is not None:
            return self._extract_onnx(image, bboxes)
        elif self.backend == "bpu" or self.backend == "opencv_bpu":
            return self._extract_bpu(image, bboxes)
        else:
            return self._extract_simple(image, bboxes)
    
    def _extract_onnx(
        self,
        image: np.ndarray,
        bboxes: np.ndarray,
    ) -> np.ndarray:
        """使用 ONNX 模型提取特征"""
        features = []
        
        for bbox in bboxes:
            # 裁剪目标区域
            crop = self._crop_and_resize(image, bbox)
            
            if crop is None:
                features.append(np.zeros(self.feature_dim))
                continue
            
            # 预处理: BGR -> RGB, 归一化, 转换为 NCHW
            input_tensor = self._preprocess(crop)
            
            # 推理
            try:
                output = self.session.run(
                    [self.output_name],
                    {self.input_name: input_tensor}
                )
                feature = output[0].flatten()
                
                # L2 归一化
                feature = feature / (np.linalg.norm(feature) + 1e-6)
                features.append(feature)
            except Exception as e:
                print(f"特征提取失败: {e}")
                features.append(np.zeros(self.feature_dim))
        
        return np.array(features)
    
    def _extract_bpu(
        self,
        image: np.ndarray,
        bboxes: np.ndarray,
    ) -> np.ndarray:
        """
        使用 BPU 模型提取特征
        
        支持三种 BPU 运行时：
        1. horizon_tc Runtime
        2. hbDNN
        3. OpenCV DNN with BPU backend
        """
        features = []
        
        for bbox in bboxes:
            # 裁剪目标区域
            crop = self._crop_and_resize(image, bbox)
            
            if crop is None:
                features.append(np.zeros(self.feature_dim))
                continue
            
            # 预处理
            input_tensor = self._preprocess(crop)
            
            feature = None
            
            # 方式1: horizon_tc Runtime
            if hasattr(self, '_bpu_runtime'):
                try:
                    import horizon_tc
                    input_data = input_tensor.astype(np.float32)
                    self._bpu_input.set_data(input_data)
                    self._bpu_runtime.run()
                    feature = self._bpu_output.get_data().flatten()
                except Exception as e:
                    print(f"⚠️  horizon_tc 推理失败: {e}")
            
            # 方式2: hbDNN
            if feature is None and hasattr(self, '_bpu_model'):
                try:
                    import hbDNN
                    input_data = input_tensor.astype(np.float32)
                    output = hbDNN.infer(self._bpu_model, input_data)
                    feature = output.flatten()
                except Exception as e:
                    print(f"⚠️  hbDNN 推理失败: {e}")
            
            # 方式3: OpenCV DNN with BPU backend
            if feature is None and hasattr(self, 'net') and self.backend == "opencv_bpu":
                try:
                    self.net.setInput(input_tensor)
                    output = self.net.forward()
                    feature = output.flatten()
                except Exception as e:
                    print(f"⚠️  OpenCV BPU 推理失败: {e}")
            
            # 回退到 ONNX
            if feature is None and self.session is not None:
                try:
                    output = self.session.run(
                        [self.output_name],
                        {self.input_name: input_tensor}
                    )
                    feature = output[0].flatten()
                except Exception as e:
                    print(f"⚠️  ONNX 回退推理失败: {e}")
            
            if feature is None:
                features.append(np.zeros(self.feature_dim))
            else:
                # L2 归一化
                feature = feature / (np.linalg.norm(feature) + 1e-6)
                features.append(feature)
        
        return np.array(features)
    
    def _extract_simple(
        self,
        image: np.ndarray,
        bboxes: np.ndarray,
    ) -> np.ndarray:
        """
        使用简单的颜色直方图特征（后备方案）
        """
        features = []
        
        for bbox in bboxes:
            crop = self._crop_and_resize(image, bbox)
            
            if crop is None:
                features.append(np.zeros(self.feature_dim))
                continue
            
            # 计算 HSV 颜色直方图
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            
            h_hist = cv2.calcHist([hsv], [0], None, [30], [0, 180])
            s_hist = cv2.calcHist([hsv], [1], None, [32], [0, 256])
            v_hist = cv2.calcHist([hsv], [2], None, [32], [0, 256])
            
            feature = np.concatenate([
                h_hist.flatten(),
                s_hist.flatten(),
                v_hist.flatten(),
            ])
            
            # 调整到目标维度
            if len(feature) < self.feature_dim:
                feature = np.pad(feature, (0, self.feature_dim - len(feature)))
            else:
                feature = feature[:self.feature_dim]
            
            # L2 归一化
            feature = feature / (np.linalg.norm(feature) + 1e-6)
            features.append(feature)
        
        return np.array(features)
    
    def _crop_and_resize(
        self,
        image: np.ndarray,
        bbox: np.ndarray,
    ) -> Optional[np.ndarray]:
        """裁剪并调整图像大小"""
        x1, y1, x2, y2 = bbox.astype(int)
        h, w = image.shape[:2]
        
        # 边界检查
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(w, x2)
        y2 = min(h, y2)
        
        if x2 <= x1 or y2 <= y1:
            return None
        
        crop = image[y1:y2, x1:x2]
        
        # 调整大小
        try:
            crop = cv2.resize(crop, (self.input_size[1], self.input_size[0]))
        except Exception:
            return None
        
        return crop
    
    def _preprocess(self, image: np.ndarray) -> np.ndarray:
        """
        图像预处理
        
        Args:
            image: BGR 图像 (H, W, 3)
            
        Returns:
            tensor: 预处理后的张量 (1, 3, H, W)
        """
        # BGR -> RGB
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 归一化到 [0, 1]
        image = image.astype(np.float32) / 255.0
        
        # 标准化 (ImageNet 均值和标准差)
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        image = (image - mean) / std
        
        # HWC -> CHW
        image = np.transpose(image, (2, 0, 1))
        
        # 添加 batch 维度
        image = np.expand_dims(image, axis=0)
        
        return np.ascontiguousarray(image)
