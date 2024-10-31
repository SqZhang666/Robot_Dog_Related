import cv2
import numpy as np



def get_img_middle(img, size=[640,480]):      # 定义一个函数，用于从给定图像中截取指定尺寸的中间部分
	top = int((img.shape[0] - size[1]) / 2)    # 计算顶部边界：纵向上，从原始图像中减去目标高度后除以 2
	bottom = top + size[1]                      # 计算底部边界：顶部边界加上目标高度
	left = int((img.shape[1] - size[0]) / 2)    # 计算左侧边界：横向上，从原始图像中减去目标宽度后除以 2
	right = left + size[0]                      # 计算右侧边界：左侧边界加上目标宽度
	return img[top:bottom, left:right]          # 返回截取后的图像（中间部分）
#这个函数接收两个参数：原始图像 img 和需要截取的区域尺寸 size。通过计算顶部、底部、左侧和右侧的边界，函数返回原始图像中心区域的截取结果。



