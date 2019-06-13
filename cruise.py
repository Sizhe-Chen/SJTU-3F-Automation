from driver import *
import cv2
import numpy as np
from math import atan as arctan
from math import asin as arcsin
from math import tan
import time
import os
import collections


def cut(value, bit=3): return round(value, bit)


def show(img):
    cv2.imshow("img", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def get_img(camera, closed_size):
    _, img = camera.read()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) #二值化
    img = img[:, drift:] #消除摄像头偏移
    img_origin = img
    img = cv2.equalizeHist(img) #对比度增强
    img = np.uint8(np.clip((1.5 * img + 10), 0, 255)) #对比度增强
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU) #阈值分割
    img = cv2.erode(cv2.dilate(img, np.ones((closed_size, closed_size))), np.ones((closed_size, closed_size))) #闭运算
    return img_origin, img


def get_error(img, up_step, black_step_max, dist_diff_max, black_depth_min, black_depth_max, zebra_edge=0):
    global dist_before, length
    # 越来越快地搜索黑线
    def search_black(increment=1):
        deviation = 0
        step = 1
        while 1:
            if img[depth][int(length/2 - deviation)] < 5: return length/2 - deviation
            if img[depth][int(length/2 + deviation)] < 5: return length/2 + deviation
            if deviation >= length / 2 - step:
                #print("\n\nNo Line in depth", depth, '\n')
                return -1
            deviation += step
            if step < black_step_max: step += increment
    # 在斑马线区域搜索要巡的线
    def search_black_zebra_edge(increment=1):
        deviation = zebra_edge
        step = 1
        left, right = 0, 0
        while deviation > step and not left*right:
            if img[depth][int(length/2 - deviation)] < 5: left = length/2 - deviation
            if img[depth][int(length/2 + deviation)] < 5: right = length/2 + deviation
            deviation -= step
            if step < black_step_max: step += increment
        left = int(length/2 - zebra_edge) if not left else left
        right = int(length/2 + zebra_edge) if not right else right
        return (left + right)/2
    
    # 从下往上搜索黑线，作为控制器输入
    keep = False
    line_deviation = -1
    depth = black_depth_max
    while line_deviation == -1:
        if not zebra_edge: line_deviation = search_black()
        else: line_deviation = search_black_zebra_edge()
        dist = line_deviation - length / 2 # 车身偏左为正
        depth -= up_step
        if depth <= black_depth_min:
            keep = True
            break
    #print('[ BlackDepth', depth + up_step, ']')

    # 根据dist微分算alpha
    if abs(dist - dist_before) <= dist_diff_max:
        alpha = arcsin((dist - dist_before) / dist_diff_max) # 车头偏右为正
    else: 
        #print('\nDist Changes too Fast\n')
        alpha = 0
    #print('[ Dist', int(dist), '] [ Alpha', cut(alpha), ']', end=' ')
    
    dist_before = dist
    return alpha, dist, depth + up_step, keep


def get_stanley_control(img, alpha, dist, motorAlphaPara, motorDistPara, motorMax, motorMin, steerAlphaPara, steerDistPara, steerMax):
    def constrain(value, threshold_max, threshold_min): return min(max(value, threshold_min), threshold_max)

    # 反比控制 motor
    if alpha == 0 and dist == 0: motor = motorMax
    if alpha == 0 and dist != 0: motor = constrain(abs(motorDistPara / dist), motorMax, motorMin)
    if alpha != 0 and dist == 0: motor = constrain(abs(motorAlphaPara / alpha), motorMax, motorMin)
    if alpha != 0 and dist != 0: motor = constrain(abs(motorAlphaPara / alpha) + abs(motorDistPara / dist), motorMax, motorMin)
    steer = constrain(alpha * steerAlphaPara + arctan(steerDistPara * dist / motor), steerMax, -steerMax)
    #print('[ StDist', cut(arctan(steerDistPara * dist / motor)), '] [ StAlpha', cut(alpha * steerAlphaPara), ']')
    
    # 保存重要参数
    text_dict = collections.OrderedDict()
    text_dict['Time'] = time.strftime("%Y-%m-%d %H-%M-%S")
    text_dict['Dist'] = int(dist)
    text_dict['Alpha'] = cut(alpha)
    text_dict['StDist'] = cut(arctan(steerDistPara * dist / motor))
    text_dict['StAlpha'] = cut(alpha * steerAlphaPara)
    text_dict['Motor'] = cut(motor)
    text_dict['Steer'] = cut(steer)
    text_dict['motorAlphaPara'] = motorAlphaPara
    text_dict['motorDistPara'] = motorDistPara
    text_dict['steerAlphaPara'] = steerAlphaPara
    text_dict['steerDistPara'] = steerDistPara
    return motor, steer, text_dict


def visualization(img, text_dict, closed_size, black_depth, doshow, dosave, dovideo):
    # 显示图片
    def show():
        cv2.imshow('image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    # 保存图片
    def save():
        OUTPUT_DIR = 'images'
        if not os.path.exists(OUTPUT_DIR): os.mkdir(OUTPUT_DIR)
        cv2.imwrite(OUTPUT_DIR + '/' + str(cut(time.time())) + '.jpg', img)
    # 显示摄像头图像，s保存，esc退出
    def video():
        while 1:
            _, img = cv2.VideoCapture(0).read()
            cv2.imshow('image', img)
            if cv2.waitKey(1) & 0xFF == ord('s'): save()
            if cv2.waitKey(1) & 0xFF == 27: break
        cv2.destroyAllWindows()

    # 不需可视化直接返回，加速
    if not doshow and not dosave and not dovideo: return

    # 加参数水印
    text_dict['closed_size'] = closed_size
    text_dict['black_depth'] = black_depth
    position = 30
    for key, value in text_dict.items():
        cv2.putText(img, key + ' = ' + str(value), (20, position), cv2.FONT_HERSHEY_PLAIN, 1.3, (128, 128, 128), 2)
        position += 30

    if doshow: show()
    if dosave: save()
    if dovideo:video()


def control(d, motor, steer):
    global time_before
    d.setStatus(motor=motor, servo=steer)
    current = time.time()
    print('[ Motor', cut(motor), '] [ Steer', cut(steer), '] [ Time', cut(current - time_before), ']\n')
    time_before = current


drift = 60               # 摄像头偏移
length = 640 - drift     # 有效图像位置
dist_before = length / 2 # 上一帧的误差
time_before = time.time()# 上一帧的时间

motorAlphaPara = 0.15    # motor关于alpha（arcsin(max=5 / motor)）的反比例系数
motorDistPara = 5        # motor关于dist（-320~320）的反比例系数
motorMax = 0.4           # motor最大值
motorMin = 0.15          # motor最小值
steerAlphaPara = 0.25    # steer关于alpha（arcsin(max=5 / motor)）的反比例系数
steerDistPara = -0.0015  # steer关于dist（-320~320）的反比例系数
steerMax = 1             # steer最大绝对值

closed_size = 11         # 闭运算的核大小，越大噪声越小，但容易丢失黑线
up_step = 1              # 发现底部没有黑线时，向上搜的像素步长
black_step_max = 5       # 搜索黑色时的最大步长
dist_diff_max = 5        # dist连续两次偏差的最大值，防止扰动带来的差值过大，alpha过大，与black_step_max有关
black_depth_min = 477    # 黑线结束搜索的位置
black_depth_max = 479    # 黑线开始搜索的位置
camera = cv2.VideoCapture(0)


def cruise():
    d = driver()
    d.setStatus(mode="speed")
    isfirst = True

    while 1:
        try:
            _, img = get_img(camera, closed_size)
            alpha, dist, black_depth, keep = get_error(img, up_step, black_step_max, dist_diff_max, black_depth_min, black_depth_max)
            if isfirst or not keep: motor, steer, text_dict = get_stanley_control(img, alpha, dist, motorAlphaPara, motorDistPara, motorMax, motorMin, steerAlphaPara, steerDistPara, steerMax)
            isfirst = False
            visualization(img, text_dict, closed_size, black_depth, doshow=False, dosave=False, dovideo=False)
            control(d, motor, steer)
        except KeyboardInterrupt: break
    
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    d.close()
    del d


if __name__ == '__main__':
    cruise()