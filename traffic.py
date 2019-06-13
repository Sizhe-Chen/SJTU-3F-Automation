from cruise import *
from detect import *


def cruise_main(d, img, isfirst, zebra_edge=0, motorMax=motorMax):
    global motor, steer, text_dict
    alpha, dist, black_depth, keep = get_error(img, up_step, black_step_max, dist_diff_max, black_depth_min, black_depth_max, zebra_edge)
    if isfirst or not keep: motor, steer, text_dict = get_stanley_control(img, alpha, dist, motorAlphaPara, motorDistPara, motorMax, motorMin, steerAlphaPara, steerDistPara, steerMax)
    visualization(img, text_dict, closed_size, black_depth, doshow=False, dosave=False, dovideo=False)
    control(d, motor, steer)


def move_away_from_zebra(d, camera, timer, closed_size, zebra_edge, zebra_motorMax):
    print("Moving away from zebra...")
    start = time.time()
    while time.time() - start <= timer:
        img_origin, img = get_img(camera, closed_size)
        cruise_main(d, img, False, zebra_edge, zebra_motorMax)
    print("Away")
    return img_origin, img


def control_open(d, motor, steer, timer):
    control(d, motor, steer)
    print("Open loop controlling...")
    time.sleep(timer)
    print("Return to closed loop")


time_for_zebra = 3     # 检测到斑马线后巡线离开的时长
time_for_turn = 1      # 看到交通标志后直走后的急转时长
steer_for_turn = 1     # 转弯幅度
motor_for_turn = 0.2   # 转弯速度

motorMax = 0.25        # 为交通标志识别配制不同的巡线参数
motorMin = 0.1
steerMax = 0.8

detect_interval = 5    # 每多少帧检测一侧
zebra_edge = 100       # 走出斑马线时，从两侧向内检测的范围-zebra_edge~zebra_edge
zebra_motorMax = 0.1   # 走出斑马线时的速度


def traffic():
    d = driver()
    d.setStatus(mode="speed")
    isfirst = True
    decisions = {1:'left', 0:'straight', -1:'right'}
    dodetect = 0
    
    while 1:
        try:
            img_origin, img = get_img(camera, closed_size)
            cruise_main(d, img, isfirst)
            isfirst = False
            dodetect = (dodetect + 1) % detect_interval
            if dodetect % detect_interval != 0: continue
            
            zebra_conf = detect(img_origin, zebra_threshold, zebra, zebra_area)
            decision, sign_conf = detect_sign(img_origin, sign_threshold, right, straight, left, sign_area)
            print('[ Detection ] [ Zebra', zebra_conf, '] [ Sign', sign_conf, ']')

            if zebra_conf > zebra_conf_threshold:
                print("[ Zebra ] [ Confidence", zebra_conf, ']')
                control_open(d, 0, 0, 3)
                img_origin, img = move_away_from_zebra(d, camera, time_for_zebra, closed_size, zebra_edge, zebra_motorMax)
   
            if sign_conf > sign_conf_threshold:
                print("[ Sign ] [ Confidence", sign_conf, '] [ Decision', decisions[decision], ']')
                if decision:
                    motor = motor_for_turn
                    steer = decision * steer_for_turn
                    control_open(d, motor, steer, time_for_turn)
                else: cruise_main(d, img, False)
        
        except KeyboardInterrupt:
            break
    d.setStatus(motor=0.0, servo=0.0, dist=0x00, mode="stop")
    d.close()
    del d


if __name__ == '__main__':
    traffic()