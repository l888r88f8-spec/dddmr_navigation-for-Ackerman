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
from trt_interface import TensorRTInference

def _img_transform(img):
    return np.array(img)

if __name__ == "__main__":

    engine_path = "../model/ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt"
    color_csv_path = "../data/colors_mapillary.csv"
    trt_inference = TensorRTInference(engine_path, color_csv_path)

    input_transform = transforms.Compose([
        transforms.ToTensor(),
        transforms.Normalize([.485, .456, .406], [.229, .224, .225]),
    ])

    img = Image.open("../data/people.png").convert('RGB')
    img = img.resize((848,424))
    img = _img_transform(img)
    img = input_transform(img)

    output_data = trt_inference.infer(img)
    reshaped_output_data = np.reshape(output_data, (1, 19, 424, 848))
    pred = np.argmax(reshaped_output_data, 1)
    #t = torch.from_numpy(reshaped_output_data)

    #pred = torch.argmax(t, 1)
    #pred = pred.cpu().data.numpy()

    predict = pred.squeeze(0)

    plt.imshow(predict)
    plt.show()


