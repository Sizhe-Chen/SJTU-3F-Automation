import cv2
import numpy as np


def match(img, template, threshold):
    res = cv2.matchTemplate(img, template, cv2.TM_CCOEFF_NORMED)
    res[res < threshold] = 0
    return np.sum(res)


def detect_sign(img, sign_threshold, right, straight, left, area):
    img = img[area[0]:area[1], area[2]:area[3]]
    similarity = {0: 0, 1: 0, 2: 0}
    
    for i, template in enumerate([right, straight, left]):
        sim = match(img, template, sign_threshold)
        if sim > similarity[i]: similarity[i] = sim
    
    final_sim = max(similarity.values())
    if not final_sim: return 0, 0.0
    return {v: k for k, v in similarity.items()}[final_sim] - 1, final_sim


def detect(img, threshold, template, area):
    return match(img[area[0]:area[1], area[2]:area[3]], template, threshold)


size = 50 # original 178*178
right = cv2.resize(cv2.imread("right.jpg", 0), (size, size))
straight = cv2.resize(cv2.imread("straight.jpg", 0), (size, size))
left = cv2.resize(cv2.imread("left.jpg", 0), (size, size))
sign_area = (0, 200, 80, 500) # 有效区域，up down left right
sign_threshold = 0.5          # 越大越容易漏过，越小越容易错误，有效值在0~1之间
sign_conf_threshold = 15      # 越大越容易漏过，越小越容易错误，有效值在10~50之间

zebra = cv2.imread("zebra.jpg", 0) # 40*275
zebra_area = (300, 420, 0, 580)    # 有效区域，up down left right
zebra_threshold = 0.6              # 越大越容易漏过，越小越容易错误，有效值在0~1之间
zebra_conf_threshold = 10          # 越大越容易漏过，越小越容易错误，有效值在10~350之间