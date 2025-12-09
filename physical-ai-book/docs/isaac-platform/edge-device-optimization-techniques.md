# Edge Device Optimization Techniques for Robotics

This document provides comprehensive techniques for optimizing robotics applications on edge devices with limited computational resources.

## Overview

Edge computing in robotics involves running AI algorithms and processing sensor data directly on the robot rather than in the cloud. This approach reduces latency, improves privacy, and enables operation in environments with limited connectivity. However, edge devices have constraints in processing power, memory, and energy consumption that require specific optimization techniques.

## Edge Device Characteristics

### Common Edge Platforms for Robotics
- **NVIDIA Jetson Series**: Jetson Nano, TX2, Xavier NX, AGX Xavier, AGX Orin
- **Raspberry Pi**: Pi 4, Pi 5 with AI capabilities
- **Google Coral**: Edge TPU-based devices
- **Intel Neural Compute Stick**: USB-based AI acceleration
- **Qualcomm Robotics RB5**: ARM-based robotics platform
- **Samsung Artik**: IoT-focused edge computing

### Resource Constraints
- **CPU**: Limited cores and clock speed
- **GPU**: Reduced parallel processing capability
- **Memory**: Limited RAM and storage
- **Power**: Battery life considerations
- **Thermal**: Heat dissipation in compact systems
- **Connectivity**: Intermittent or low-bandwidth networks

## Optimization Strategies

### 1. Model Optimization

#### Quantization
```python
# model_quantization.py
import torch
import torch.quantization as quant
import numpy as np

class ModelQuantizer:
    """
    Quantize neural networks for edge deployment
    """
    def __init__(self):
        self.quantization_config = {
            'static': quant.default_qconfig,
            'dynamic': quant.default_dynamic_qconfig,
            'qat': quant.default_qat_qconfig()
        }

    def static_quantization(self, model, calibration_data_loader):
        """
        Apply static quantization with calibration data
        """
        # Set model to evaluation mode
        model.eval()

        # Fuse conv+bn+relu layers for better quantization
        model = self._fuse_model_layers(model)

        # Specify quantization configuration
        model.qconfig = quant.default_qconfig

        # Prepare model for quantization
        quant_model = quant.prepare(model)

        # Calibrate the model
        with torch.no_grad():
            for data, _ in calibration_data_loader:
                quant_model(data)

        # Convert to quantized model
        quantized_model = quant.convert(quant_model)

        return quantized_model

    def dynamic_quantization(self, model):
        """
        Apply dynamic quantization (weights quantized, activations float)
        """
        quantized_model = quant.quantize_dynamic(
            model,
            {torch.nn.Linear, torch.nn.LSTM, torch.nn.GRU},
            dtype=torch.qint8
        )
        return quantized_model

    def quantization_aware_training(self, model, train_loader, num_epochs=10):
        """
        Apply quantization-aware training
        """
        # Set model to training mode for QAT
        model.train()

        # Fuse layers
        model = self._fuse_model_layers(model)

        # Set QAT configuration
        model.qconfig = quant.default_qat_qconfig()

        # Prepare model for QAT
        qat_model = quant.prepare_qat(model, inplace=False)

        # Train the model (simplified)
        optimizer = torch.optim.SGD(qat_model.parameters(), lr=0.001)
        criterion = torch.nn.CrossEntropyLoss()

        for epoch in range(num_epochs):
            for batch_idx, (data, target) in enumerate(train_loader):
                optimizer.zero_grad()
                output = qat_model(data)
                loss = criterion(output, target)
                loss.backward()
                optimizer.step()

        # Convert to inference model
        qat_model.eval()
        quantized_model = quant.convert(qat_model)

        return quantized_model

    def _fuse_model_layers(self, model):
        """
        Fuse layers for better quantization performance
        """
        import torch.nn.utils.fusion as fusion
        # Example fusion for common patterns
        for module in model.modules():
            if type(module) == torch.nn.Conv2d:
                # Fuse Conv-BN-ReLU if present
                pass
        return model

    def tensorrt_optimization(self, model_path, precision="fp16"):
        """
        Optimize model using TensorRT (for NVIDIA devices)
        """
        import tensorrt as trt
        import pycuda.driver as cuda

        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        builder = trt.Builder(TRT_LOGGER)
        network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
        parser = trt.OnnxParser(network, TRT_LOGGER)

        with open(model_path, 'rb') as model_file:
            if not parser.parse(model_file.read()):
                for error in range(parser.num_errors):
                    print(parser.get_error(error))

        config = builder.create_builder_config()

        if precision == "fp16":
            config.set_flag(trt.BuilderFlag.FP16)
        elif precision == "int8":
            config.set_flag(trt.BuilderFlag.INT8)
            # Add calibration for INT8

        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 30)  # 1GB

        serialized_engine = builder.build_serialized_network(network, config)

        return serialized_engine
```

#### Model Pruning
```python
# model_pruning.py
import torch
import torch.nn.utils.prune as prune
import numpy as np

class ModelPruner:
    """
    Prune neural networks to reduce model size and computation
    """
    def __init__(self):
        pass

    def magnitude_pruning(self, model, pruning_ratio=0.2):
        """
        Prune weights based on magnitude
        """
        parameters_to_prune = []
        for name, module in model.named_modules():
            if isinstance(module, torch.nn.Conv2d) or isinstance(module, torch.nn.Linear):
                parameters_to_prune.append((module, 'weight'))

        parameters_to_prune = tuple(parameters_to_prune)

        # Apply global pruning
        prune.global_unstructured(
            parameters_to_prune,
            pruning_method=prune.L1Unstructured,
            amount=pruning_ratio
        )

        return model

    def iterative_pruning(self, model, train_loader, val_loader, initial_ratio=0.1,
                         target_ratio=0.8, num_iterations=10):
        """
        Iteratively prune and retrain model
        """
        current_ratio = initial_ratio

        for iteration in range(num_iterations):
            # Prune model
            self.magnitude_pruning(model, current_ratio)

            # Retrain model
            self._retrain_model(model, train_loader)

            # Validate performance
            accuracy = self._validate_model(model, val_loader)

            print(f"Iteration {iteration + 1}: Pruning ratio = {current_ratio:.2f}, Accuracy = {accuracy:.4f}")

            # Increase pruning ratio for next iteration
            current_ratio = min(current_ratio * 1.2, target_ratio)

        return model

    def _retrain_model(self, model, train_loader):
        """
        Retrain model after pruning
        """
        model.train()
        optimizer = torch.optim.Adam(model.parameters(), lr=0.001)

        for batch_idx, (data, target) in enumerate(train_loader):
            optimizer.zero_grad()
            output = model(data)
            loss = torch.nn.functional.cross_entropy(output, target)
            loss.backward()
            optimizer.step()

    def _validate_model(self, model, val_loader):
        """
        Validate model performance
        """
        model.eval()
        correct = 0
        total = 0

        with torch.no_grad():
            for data, target in val_loader:
                output = model(data)
                pred = output.argmax(dim=1, keepdim=True)
                correct += pred.eq(target.view_as(pred)).sum().item()
                total += target.size(0)

        return correct / total
```

### 2. Computational Optimization

#### Efficient Neural Architectures
```python
# efficient_architectures.py
import torch
import torch.nn as nn

class MobileNetV3Block(nn.Module):
    """
    Efficient MobileNetV3 block for edge devices
    """
    def __init__(self, inp, hidden_dim, out, kernel_size, stride, use_se, use_hs):
        super(MobileNetV3Block, self).__init__()
        assert stride in [1, 2]

        self.identity = stride == 1 and inp == out

        if inp == hidden_dim:
            self.conv = nn.Sequential(
                # dw
                nn.Conv2d(hidden_dim, hidden_dim, kernel_size, stride, (kernel_size - 1) // 2, groups=hidden_dim, bias=False),
                nn.BatchNorm2d(hidden_dim),
                nn.Hardswish() if use_hs else nn.ReLU(inplace=True),
                # Squeeze-and-Excite
                SqueezeExcite(hidden_dim) if use_se else nn.Identity(),
                # pw-linear
                nn.Conv2d(hidden_dim, out, 1, 1, 0, bias=False),
                nn.BatchNorm2d(out),
            )
        else:
            self.conv = nn.Sequential(
                # pw
                nn.Conv2d(inp, hidden_dim, 1, 1, 0, bias=False),
                nn.BatchNorm2d(hidden_dim),
                nn.Hardswish() if use_hs else nn.ReLU(inplace=True),
                # dw
                nn.Conv2d(hidden_dim, hidden_dim, kernel_size, stride, (kernel_size - 1) // 2, groups=hidden_dim, bias=False),
                nn.BatchNorm2d(hidden_dim),
                SqueezeExcite(hidden_dim) if use_se else nn.Identity(),
                nn.Hardswish() if use_hs else nn.ReLU(inplace=True),
                # pw-linear
                nn.Conv2d(hidden_dim, out, 1, 1, 0, bias=False),
                nn.BatchNorm2d(out),
            )

    def forward(self, x):
        if self.identity:
            return x + self.conv(x)
        else:
            return self.conv(x)

class SqueezeExcite(nn.Module):
    """
    Squeeze and Excitation block
    """
    def __init__(self, dim, squeeze_factor=4):
        super().__init__()
        squeeze_dim = max(dim // squeeze_factor, 1)
        self.fc1 = nn.Conv2d(dim, squeeze_dim, 1, bias=True)
        self.act = nn.ReLU(inplace=True)
        self.fc2 = nn.Conv2d(squeeze_dim, dim, 1, bias=True)
        self.gate = nn.Hardsigmoid(inplace=True)

    def forward(self, x):
        scale = torch.adaptive_avg_pool2d(x, 1)
        scale = self.fc1(scale)
        scale = self.act(scale)
        scale = self.fc2(scale)
        return x * self.gate(scale)

class EfficientNetLite(nn.Module):
    """
    Lightweight EfficientNet for edge deployment
    """
    def __init__(self, num_classes=1000, width_mult=1.0):
        super(EfficientNetLite, self).__init__()

        # Define the architecture with fewer parameters
        self.features = nn.Sequential(
            # Initial conv layer
            nn.Conv2d(3, 32, 3, 2, 1, bias=False),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),

            # MobileNetV3 blocks
            MobileNetV3Block(32, 32, 16, 3, 1, False, True),  # Reduced channels
            MobileNetV3Block(16, 96, 24, 3, 2, False, True),  # Reduced channels
            MobileNetV3Block(24, 144, 24, 3, 1, False, True), # Reduced channels
            MobileNetV3Block(24, 144, 40, 5, 2, True, True),  # Reduced channels
            MobileNetV3Block(40, 240, 40, 5, 1, True, True),  # Reduced channels
            MobileNetV3Block(40, 240, 80, 3, 2, False, True), # Reduced channels
            MobileNetV3Block(80, 480, 80, 3, 1, False, True), # Reduced channels
            MobileNetV3Block(80, 480, 112, 3, 1, True, True), # Reduced channels
            MobileNetV3Block(112, 672, 160, 5, 2, True, True), # Reduced channels
            MobileNetV3Block(160, 960, 160, 5, 1, True, True), # Reduced channels
        )

        self.avgpool = nn.AdaptiveAvgPool2d(1)
        self.classifier = nn.Sequential(
            nn.Linear(160, 1280),  # Reduced from original
            nn.Hardswish(inplace=True),
            nn.Dropout(p=0.2),
            nn.Linear(1280, num_classes),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.avgpool(x)
        x = torch.flatten(x, 1)
        x = self.classifier(x)
        return x
```

### 3. Memory Optimization

#### Memory-Efficient Data Loading
```python
# memory_optimization.py
import torch
import numpy as np
from torch.utils.data import Dataset, DataLoader
import gc

class MemoryEfficientDataset(Dataset):
    """
    Dataset class with memory-efficient loading
    """
    def __init__(self, data_paths, transform=None, load_in_memory=False):
        self.data_paths = data_paths
        self.transform = transform
        self.load_in_memory = load_in_memory

        if load_in_memory:
            self.data_cache = {}
            self._preload_data()
        else:
            self.data_cache = None

    def _preload_data(self):
        """
        Preload data into memory if specified
        """
        for i, path in enumerate(self.data_paths):
            # Load data and store in cache
            self.data_cache[i] = self._load_single_item(path)

    def _load_single_item(self, path):
        """
        Load a single data item
        """
        # Implementation depends on data type
        # For example, for images:
        # return Image.open(path).convert('RGB')
        pass

    def __len__(self):
        return len(self.data_paths)

    def __getitem__(self, idx):
        if self.data_cache is not None and idx in self.data_cache:
            data = self.data_cache[idx]
        else:
            data = self._load_single_item(self.data_paths[idx])

        if self.transform:
            data = self.transform(data)

        return data

class MemoryManager:
    """
    Manage memory usage during inference
    """
    def __init__(self, max_memory_mb=1000):
        self.max_memory_mb = max_memory_mb
        self.current_memory_mb = 0

    def optimize_tensor_memory(self, tensor):
        """
        Optimize tensor memory usage
        """
        # Convert to appropriate precision
        if tensor.dtype == torch.float64:
            tensor = tensor.float()  # Convert from double to float

        # Move to appropriate device
        if torch.cuda.is_available():
            tensor = tensor.cuda()

        return tensor

    def clear_cache(self):
        """
        Clear PyTorch cache to free memory
        """
        if torch.cuda.is_available():
            torch.cuda.empty_cache()

        # Force garbage collection
        gc.collect()

    def monitor_memory_usage(self):
        """
        Monitor current memory usage
        """
        if torch.cuda.is_available():
            allocated = torch.cuda.memory_allocated() / (1024**2)  # MB
            reserved = torch.cuda.memory_reserved() / (1024**2)    # MB
            return {'allocated_mb': allocated, 'reserved_mb': reserved}
        else:
            # For CPU, return a placeholder
            return {'allocated_mb': 0, 'reserved_mb': 0}
```

### 4. Real-Time Optimization

#### Asynchronous Processing Pipeline
```python
# async_pipeline.py
import asyncio
import threading
import queue
import time
from typing import Callable, Any, Optional
import numpy as np

class AsyncProcessingPipeline:
    """
    Asynchronous processing pipeline for real-time edge inference
    """
    def __init__(self, max_concurrent_tasks=2):
        self.max_concurrent_tasks = max_concurrent_tasks
        self.input_queue = queue.Queue(maxsize=10)  # Bounded queue to prevent memory buildup
        self.output_queue = queue.Queue(maxsize=10)
        self.processing_tasks = []
        self.is_running = False

    async def process_frame_async(self, frame, model_func: Callable) -> Any:
        """
        Process a single frame asynchronously
        """
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(None, model_func, frame)

    def start_pipeline(self, model_func: Callable):
        """
        Start the asynchronous processing pipeline
        """
        self.is_running = True
        self.pipeline_task = asyncio.create_task(
            self._pipeline_worker(model_func)
        )

    async def _pipeline_worker(self, model_func: Callable):
        """
        Worker that processes frames from the input queue
        """
        sem = asyncio.Semaphore(self.max_concurrent_tasks)

        async def process_item(item):
            async with sem:
                result = await self.process_frame_async(item['data'], model_func)
                item['result'] = result
                self.output_queue.put(item)

        while self.is_running:
            try:
                # Get item from input queue with timeout
                item = self.input_queue.get_nowait()
                # Process item asynchronously
                asyncio.create_task(process_item(item))
            except queue.Empty:
                await asyncio.sleep(0.001)  # Small delay to prevent busy waiting

    def add_frame(self, frame, metadata=None):
        """
        Add a frame to the processing pipeline
        """
        item = {
            'data': frame,
            'timestamp': time.time(),
            'metadata': metadata or {}
        }

        try:
            self.input_queue.put_nowait(item)
            return True
        except queue.Full:
            # Queue is full, drop the frame
            return False

    def get_result(self, timeout=None):
        """
        Get a processed result
        """
        try:
            result = self.output_queue.get(timeout=timeout)
            return result
        except queue.Empty:
            return None

    def stop_pipeline(self):
        """
        Stop the processing pipeline
        """
        self.is_running = False
        if hasattr(self, 'pipeline_task'):
            self.pipeline_task.cancel()
```

### 5. Power Optimization

#### Power-Aware Inference
```python
# power_optimization.py
import time
import psutil
import threading
from typing import Dict, Callable
import torch

class PowerAwareInference:
    """
    Power-aware inference with dynamic adjustment
    """
    def __init__(self, battery_threshold=20.0, cpu_usage_limit=80.0):
        self.battery_threshold = battery_threshold
        self.cpu_usage_limit = cpu_usage_limit
        self.power_monitoring = False
        self.monitoring_thread = None

    def start_power_monitoring(self):
        """
        Start power monitoring in a separate thread
        """
        self.power_monitoring = True
        self.monitoring_thread = threading.Thread(target=self._monitor_power_usage)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def _monitor_power_usage(self):
        """
        Monitor system power usage
        """
        while self.power_monitoring:
            # Get battery level
            battery = psutil.sensors_battery()
            if battery:
                battery_percent = battery.percent
                is_charging = battery.power_plugged

                # Get CPU usage
                cpu_percent = psutil.cpu_percent(interval=1)

                # Adjust inference parameters based on power state
                if battery_percent < self.battery_threshold and not is_charging:
                    self._reduce_power_consumption()
                elif cpu_percent > self.cpu_usage_limit:
                    self._throttle_inference()

            time.sleep(5)  # Check every 5 seconds

    def _reduce_power_consumption(self):
        """
        Reduce power consumption by lowering performance
        """
        # This could involve:
        # - Reducing model precision
        # - Lowering frame rate
        # - Using smaller model variants
        print("Reducing power consumption due to low battery")

    def _throttle_inference(self):
        """
        Throttle inference to reduce CPU usage
        """
        # This could involve:
        # - Adding delays between inferences
        # - Skipping frames
        # - Using simpler models
        print("Throttling inference due to high CPU usage")

    def adaptive_inference(self, model, input_data, power_mode="balanced"):
        """
        Perform inference with power-aware adaptations
        """
        if power_mode == "power_saving":
            # Use quantized model, lower resolution, etc.
            return self._power_saving_inference(model, input_data)
        elif power_mode == "performance":
            # Use full model, higher resolution
            return self._performance_inference(model, input_data)
        else:  # balanced
            # Use medium settings
            return self._balanced_inference(model, input_data)

    def _power_saving_inference(self, model, input_data):
        """
        Power-saving inference implementation
        """
        # Reduce input resolution
        reduced_input = self._reduce_resolution(input_data, factor=0.5)

        # Use quantized model if available
        if hasattr(model, 'quantized_model'):
            return model.quantized_model(reduced_input)
        else:
            return model(reduced_input)

    def _performance_inference(self, model, input_data):
        """
        Performance-focused inference implementation
        """
        return model(input_data)

    def _balanced_inference(self, model, input_data):
        """
        Balanced inference implementation
        """
        # Use medium resolution and settings
        reduced_input = self._reduce_resolution(input_data, factor=0.75)
        return model(reduced_input)

    def _reduce_resolution(self, data, factor=0.5):
        """
        Reduce resolution of input data
        """
        if isinstance(data, torch.Tensor):
            # Use PyTorch's interpolation for tensor data
            import torch.nn.functional as F
            h, w = data.shape[-2], data.shape[-1]
            new_h, new_w = int(h * factor), int(w * factor)
            return F.interpolate(data, size=(new_h, new_w), mode='bilinear', align_corners=False)
        else:
            # For other data types, implement accordingly
            return data
```

### 6. Deployment Optimization

#### Model Serving on Edge
```python
# edge_serving.py
import torch
import numpy as np
import time
from typing import Dict, Any, List
import json

class EdgeModelServer:
    """
    Model serving framework optimized for edge devices
    """
    def __init__(self, model_path: str, device: str = None):
        self.model_path = model_path
        self.device = device or ('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = None
        self.load_time = 0
        self.warmup_complete = False

    def load_model(self, optimization_level: str = "balanced"):
        """
        Load and optimize model for edge deployment
        """
        start_time = time.time()

        # Load the model
        self.model = torch.jit.load(self.model_path)
        self.model.to(self.device)
        self.model.eval()

        # Apply optimizations based on level
        if optimization_level == "size":
            self.model = self._optimize_for_size(self.model)
        elif optimization_level == "speed":
            self.model = self._optimize_for_speed(self.model)
        elif optimization_level == "balanced":
            self.model = self._optimize_balanced(self.model)

        self.load_time = time.time() - start_time

    def _optimize_for_size(self, model):
        """
        Optimize model for minimal size
        """
        # Apply aggressive quantization
        model = torch.quantization.quantize_dynamic(
            model, {torch.nn.Linear, torch.nn.Conv2d}, dtype=torch.qint8
        )
        return model

    def _optimize_for_speed(self, model):
        """
        Optimize model for maximum speed
        """
        # Use TensorRT if available
        if self.device == 'cuda':
            try:
                import tensorrt as trt
                # Apply TensorRT optimization
                pass
            except ImportError:
                pass
        return model

    def _optimize_balanced(self, model):
        """
        Balance size and speed optimization
        """
        # Apply moderate quantization
        model = torch.quantization.quantize_dynamic(
            model, {torch.nn.Linear}, dtype=torch.qint8
        )
        return model

    def warmup(self, sample_input, num_runs=10):
        """
        Warm up the model to stabilize performance
        """
        self.model.eval()
        with torch.no_grad():
            for _ in range(num_runs):
                _ = self.model(sample_input.to(self.device))

        self.warmup_complete = True

    def predict(self, input_data, return_time=False) -> Dict[str, Any]:
        """
        Perform inference with timing and error handling
        """
        if not self.warmup_complete:
            raise RuntimeError("Model must be warmed up before inference")

        start_time = time.time()

        try:
            with torch.no_grad():
                # Move input to device
                if isinstance(input_data, torch.Tensor):
                    input_tensor = input_data.to(self.device)
                else:
                    input_tensor = torch.tensor(input_data, device=self.device)

                # Perform inference
                output = self.model(input_tensor)

                # Move output back to CPU for return
                if isinstance(output, torch.Tensor):
                    output = output.cpu()

                inference_time = time.time() - start_time

                result = {
                    'output': output,
                    'inference_time': inference_time,
                    'success': True
                }

                if return_time:
                    result['total_time'] = inference_time

                return result

        except Exception as e:
            return {
                'error': str(e),
                'success': False,
                'inference_time': time.time() - start_time
            }

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the loaded model
        """
        return {
            'model_path': self.model_path,
            'device': self.device,
            'load_time': self.load_time,
            'model_size_mb': self._get_model_size_mb(),
            'parameters': self._count_parameters()
        }

    def _get_model_size_mb(self) -> float:
        """
        Get model size in MB
        """
        import os
        size_bytes = os.path.getsize(self.model_path)
        return size_bytes / (1024 * 1024)

    def _count_parameters(self) -> int:
        """
        Count model parameters
        """
        if self.model:
            return sum(p.numel() for p in self.model.parameters())
        return 0
```

### 7. Monitoring and Profiling

#### Performance Monitoring
```python
# performance_monitoring.py
import time
import psutil
import torch
from typing import Dict, List
import threading
import json

class EdgePerformanceMonitor:
    """
    Monitor performance metrics on edge devices
    """
    def __init__(self):
        self.metrics_history = []
        self.monitoring = False
        self.monitoring_thread = None
        self.start_time = time.time()

    def start_monitoring(self, interval=1.0):
        """
        Start performance monitoring
        """
        self.monitoring = True
        self.monitoring_thread = threading.Thread(
            target=self._monitor_loop,
            args=(interval,)
        )
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()

    def _monitor_loop(self, interval):
        """
        Monitoring loop that collects metrics
        """
        while self.monitoring:
            metrics = self._collect_metrics()
            self.metrics_history.append(metrics)

            # Keep only recent history to prevent memory buildup
            if len(self.metrics_history) > 1000:
                self.metrics_history = self.metrics_history[-500:]

            time.sleep(interval)

    def _collect_metrics(self) -> Dict:
        """
        Collect current system metrics
        """
        metrics = {
            'timestamp': time.time(),
            'uptime': time.time() - self.start_time,
            'cpu_percent': psutil.cpu_percent(),
            'memory_percent': psutil.virtual_memory().percent,
            'memory_available_mb': psutil.virtual_memory().available / (1024**2),
            'disk_percent': psutil.disk_usage('/').percent,
            'temperature': self._get_temperature(),
            'inference_count': getattr(self, 'inference_count', 0)
        }

        # Add GPU metrics if available
        if torch.cuda.is_available():
            gpu_metrics = {
                'gpu_memory_allocated_mb': torch.cuda.memory_allocated() / (1024**2),
                'gpu_memory_reserved_mb': torch.cuda.memory_reserved() / (1024**2),
                'gpu_utilization_percent': self._get_gpu_utilization()
            }
            metrics.update(gpu_metrics)

        return metrics

    def _get_temperature(self) -> float:
        """
        Get system temperature (platform-specific)
        """
        try:
            temps = psutil.sensors_temperatures()
            if 'coretemp' in temps:
                return temps['coretemp'][0].current
            elif 'cpu_thermal' in temps:
                return temps['cpu_thermal'][0].current
        except:
            pass
        return -1.0  # Unable to get temperature

    def _get_gpu_utilization(self) -> float:
        """
        Get GPU utilization (NVIDIA-specific)
        """
        try:
            import subprocess
            result = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu', '--format=csv,noheader,nounits'],
                                  capture_output=True, text=True)
            if result.returncode == 0:
                return float(result.stdout.strip())
        except:
            pass
        return -1.0

    def get_performance_summary(self) -> Dict:
        """
        Get performance summary from collected metrics
        """
        if not self.metrics_history:
            return {}

        recent_metrics = self.metrics_history[-100:] if len(self.metrics_history) > 100 else self.metrics_history

        return {
            'avg_cpu_percent': sum(m['cpu_percent'] for m in recent_metrics) / len(recent_metrics),
            'avg_memory_percent': sum(m['memory_percent'] for m in recent_metrics) / len(recent_metrics),
            'max_cpu_percent': max(m['cpu_percent'] for m in recent_metrics),
            'max_memory_percent': max(m['memory_percent'] for m in recent_metrics),
            'current_temperature': recent_metrics[-1]['temperature'] if recent_metrics else -1,
            'total_uptime': time.time() - self.start_time,
            'metric_count': len(recent_metrics)
        }

    def stop_monitoring(self):
        """
        Stop performance monitoring
        """
        self.monitoring = False
        if self.monitoring_thread:
            self.monitoring_thread.join()

    def save_metrics(self, filename: str):
        """
        Save collected metrics to file
        """
        with open(filename, 'w') as f:
            json.dump(self.metrics_history, f, indent=2)
```

## Best Practices for Edge Deployment

### 1. Progressive Enhancement
- Start with a lightweight model and enhance as resources allow
- Implement fallback mechanisms for resource-constrained scenarios
- Use model cascading (simple model filters, complex model validates)

### 2. Adaptive Inference
- Adjust model complexity based on current system load
- Implement quality-of-service scaling
- Use dynamic batching based on available resources

### 3. Caching and Precomputation
- Cache frequently computed results
- Precompute static transformations
- Use lookup tables for expensive operations

### 4. Network Optimization
- Implement model compression for network transfer
- Use differential updates to minimize data transfer
- Cache models locally when possible

## Testing and Validation

### 1. Resource Constraint Testing
- Test under various memory constraints
- Validate performance under thermal limits
- Test with different power profiles

### 2. Robustness Testing
- Test with degraded sensor inputs
- Validate behavior under network disruptions
- Test graceful degradation when resources are limited

## Troubleshooting Common Issues

### Performance Issues:
1. **High Memory Usage**: Implement memory pooling and proper cleanup
2. **Slow Inference**: Optimize model architecture and use appropriate precision
3. **Thermal Throttling**: Implement thermal management and power optimization
4. **Battery Drain**: Optimize power consumption and implement duty cycling

### Verification Steps:
1. Profile memory and CPU usage during operation
2. Test model performance under different load conditions
3. Validate accuracy is maintained after optimization
4. Monitor thermal and power characteristics

## Resources and Further Reading

- [TensorRT Optimization Guide](https://docs.nvidia.com/deeplearning/tensorrt/developer-guide/index.html)
- [PyTorch Mobile Deployment](https://pytorch.org/mobile/home/)
- [Edge AI Hardware Comparison](https://www.nvidia.com/en-us/deep-learning-ai/industries/robotics/)
- [Model Optimization Techniques](https://pytorch.org/tutorials/recipes/recipes.html#quantization)
- [Efficient Neural Network Design](https://arxiv.org/abs/1905.02244)