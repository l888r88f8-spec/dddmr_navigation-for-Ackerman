import numpy as np
import pycuda.driver as cuda
import pycuda.autoinit
import tensorrt as trt
import time
import cv2
import matplotlib.pyplot as plt

# for color map csv
import csv
from typing import Optional
from PIL import Image
import torch
from torchvision import transforms

class TensorRTInference:
    def __init__(self, engine_path, color_csv_path):

        self.cfx = cuda.Device(0).make_context()

        self.logger = trt.Logger(trt.Logger.INFO)
        self.runtime = trt.Runtime(self.logger)
        self.engine = self.load_engine(engine_path)
        self.context = self.engine.create_execution_context()

        # Allocate buffers
        self.inputs, self.outputs, self.bindings, self.stream = self.allocate_buffers(self.engine)
        # color map
        self.color_map = self.readColorMap(color_csv_path)

    def load_engine(self, engine_path):
        with open(engine_path, "rb") as f:
            engine = self.runtime.deserialize_cuda_engine(f.read())
        return engine

    class HostDeviceMem:
        def __init__(self, host_mem, device_mem):
            self.host = host_mem
            self.device = device_mem

    def allocate_buffers(self, engine):
        inputs, outputs, bindings = [], [], []
        stream = cuda.Stream()

        for i in range(engine.num_io_tensors):
            tensor_name = engine.get_tensor_name(i)
            size = trt.volume(engine.get_tensor_shape(tensor_name))
            dtype = trt.nptype(engine.get_tensor_dtype(tensor_name))

            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)

            # Append the device buffer address to device bindings
            bindings.append(int(device_mem))

            # Append to the appropiate input/output list
            if engine.get_tensor_mode(tensor_name) == trt.TensorIOMode.INPUT:
                inputs.append(self.HostDeviceMem(host_mem, device_mem))
            else:
                outputs.append(self.HostDeviceMem(host_mem, device_mem))

        return inputs, outputs, bindings, stream

    def infer(self, input_data):

        self.cfx.push()
        # Transfer input data to device
        np.copyto(self.inputs[0].host, input_data.ravel())
        cuda.memcpy_htod_async(self.inputs[0].device, self.inputs[0].host, self.stream)

        # Set tensor address
        for i in range(self.engine.num_io_tensors):
            self.context.set_tensor_address(self.engine.get_tensor_name(i), self.bindings[i])

        # Run inference
        self.context.execute_async_v3(stream_handle=self.stream.handle)

        # Transfer predictions back
        cuda.memcpy_dtoh_async(self.outputs[0].host, self.outputs[0].device, self.stream)

        # Synchronize the stream
        self.stream.synchronize()

        self.cfx.pop()

        return self.outputs[0].host

    def readColorMap(self, csv_path: str) -> Optional[np.ndarray]:
        """
        :param csv_path: Path to a csv file with color map
        :return: Color map or None if the path is empty
        """
        if len(csv_path) == 0:
            return None

        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile, delimiter=';')
            color_map = None
            for line in reader:
                temp = np.array(tuple(map(int, line['color'].split()))[::-1])
                if color_map is None:
                    color_map = temp[np.newaxis, :]
                else:
                    color_map = np.concatenate((color_map, temp[np.newaxis, :]))
        return color_map


    def index2Color(self, indexes: np.ndarray, color_map: np.ndarray) -> np.ndarray:
        """
        Numpy vectorized index to color conversion function
        :param indexes: Index mask predicted by network
        :param color_map: Numpy array. I-th element is a color corresponding with index mask's index
        :return: Color mask of the same size as the index mask
        """
        def map_color(index):
            return color_map[index]

        colored_mask = map_color(indexes)
        return colored_mask

    def getColorMask(self, image: np.ndarray, index_mask: np.ndarray, alpha: float) -> np.ndarray:
        """
        :param image: Image to inference
        :param index_mask: Index mask of the `image` from `get_index_mask` method
        :param alpha: Mixin coefficient of image and color mask
        :return: Returns color mask of the same size as the original image
        """
        original_width, original_height = image.shape[1], image.shape[0]
        assert self.color_map is not None, 'Color map .csv path was not specified'
        color_map = self.index2Color(index_mask, self.color_map).astype(np.float32)
        # color_map = cv2.resize(color_map, (original_width, original_height), interpolation=cv2.INTER_LINEAR)
        # image = image.astype(np.float32)
        # image = cv2.resize(image, (self.net_w, self.net_h))
        result = alpha * image + (1 - alpha) * color_map
        return result.astype(np.uint8)

    def getColor(self, image: np.ndarray, index_mask: np.ndarray) -> np.ndarray:
        """
        :param image: Image to inference
        :param index_mask: Index mask of the `image` from `get_index_mask` method
        :param alpha: Mixin coefficient of image and color mask
        :return: Returns color mask of the same size as the original image
        """
        original_width, original_height = image.shape[1], image.shape[0]
        assert self.color_map is not None, 'Color map .csv path was not specified'
        color_map = self.index2Color(index_mask, self.color_map).astype(np.float32)
        # color_map = cv2.resize(color_map, (original_width, original_height), interpolation=cv2.INTER_LINEAR)
        # image = image.astype(np.float32)
        # image = cv2.resize(image, (self.net_w, self.net_h))
        # result = color_map
        return color_map.astype(np.uint8)
