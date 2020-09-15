import socket
import threading
import time
import datetime

import cv2
import numpy as np

from .stats import Stats


class Tello:
    def __init__(self, te_ip: str = '192.168.10.1', debug: bool = True):
        # 在8889上打开本地UDP端口以进行无人机通信
        self.local_ip = ''
        self.local_port = 8889
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.local_ip, self.local_port))

        # 设置无人机IP和端口信息
        self.te_ip = te_ip
        self.te_port = 8889
        self.te_address = (self.te_ip, self.te_port)
        self.log = []

        # 初始化响应线程
        self.receive_thread = threading.Thread(target=self._receive_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        # 本项目运行时选项
        self.stream_state = False
        self.camera_state = False
        self.color_state = False
        self.now_color = 0
        self.MAX_TIME_OUT = 15.0
        self.debug = debug

        # 将无人机设置为命令模式
        self.command()

    def send_command(self, command: str, query: bool = False):
        # 为出站命令创建新的日志条目
        self.log.append(Stats(command, len(self.log)))

        # 向无人机发送命令
        self.socket.sendto(command.encode('utf-8'), self.te_address)
        # 显示确认消息
        if self.debug is True:
            print('Send Command: {}'.format(command))

        # 检查命令是否超时（基于MAX_TIME_OUT中的值）
        start = time.time()
        while not self.log[-1].got_response():  # 在日志中未收到任何响应的情况下运行
            now = time.time()
            difference = now - start
            if difference > self.MAX_TIME_OUT:
                print('Connect Time Out!')
                break

        # 打印出无人机响应
        if self.debug is True and query is False:
            print('Response: {}'.format(self.log[-1].get_response()))

    def _receive_thread(self):
        while True:
            # 检查无人机响应，引发套接字错误
            try:
                self.response, ip = self.socket.recvfrom(1024)
                self.log[-1].add_response(self.response)
            except socket.error as exc:
                print('Error: {}'.format(exc))

    def _video_thread(self):
        # 创建流捕获对象
        cap = cv2.VideoCapture('udp://' + self.te_ip + ':11111')

        while self.stream_state:
            ret, frame = cap.read()
            cv2.imshow('DJI Tello', frame)

            k = cv2.waitKey(1) & 0xFF

            # 如果按Esc键，视频流关闭
            if k == 27:
                break

            # 如果按F1键，截图到当前位置
            if k == 0 or self.camera_state:
                png_name = datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '.png'
                cv2.imwrite(png_name, frame)
                self.camera_state = False

            # 识别当前颜色
            if self.color_state:
                self.detect_color(frame)
                self.color_state = False

        cap.release()
        cv2.destroyAllWindows()

    def wait(self, delay: float):
        # 显示等待消息
        if self.debug is True:
            print('Wait {} Seconds...'.format(delay))

        # 日志条目增加了延迟
        self.log.append(Stats('wait', len(self.log)))
        # 延迟激活
        time.sleep(delay)

    def detect_color(self, frame):
        # frame = cv2.imread("test.jpg")
        hue_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        low_red_range1 = np.array([110, 43, 0])
        high_red_range1 = np.array([180, 255, 255])
        threhold_red1 = cv2.inRange(hue_image, low_red_range1, high_red_range1)
        res_red1 = cv2.bitwise_and(frame, frame, mask=threhold_red1)

        low_red_range2 = np.array([0, 43, 0])
        high_red_range2 = np.array([10, 255, 255])
        threhold_red2 = cv2.inRange(hue_image, low_red_range2, high_red_range2)
        res_red2 = cv2.bitwise_and(frame, frame, mask=threhold_red2)

        threhold_red = threhold_red1 + threhold_red2
        res_red = res_red1 + res_red2

        low_green_range = np.array([35, 43, 46])
        high_green_range = np.array([77, 255, 255])
        threhold_green = cv2.inRange(hue_image, low_green_range, high_green_range)
        res_green = cv2.bitwise_and(frame, frame, mask=threhold_green)

        res = res_red + res_green
        if (cv2.countNonZero(threhold_green) > 0.5 * np.size(threhold_green)):
            self.now_color = 'green'
        elif ((cv2.countNonZero(threhold_red) > 0.5 * np.size(threhold_red)) & (
                cv2.countNonZero(threhold_red) < 0.7 * np.size(threhold_red))):
            self.now_color = 'red'
        else:
            self.now_color = 'none'
            # color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        return self.now_color, res

    def get_log(self):
        return self.log

    def take_picture(self):
        """拍照"""
        self.camera_state = True

    def identify_color(self):
        """识别当前颜色(红色或绿色)"""
        self.color_state = True
        time.sleep(0.5)
        return self.now_color

    # 以下命令强烈建议配合官方SDK食用
    # https://www.ryzerobotics.com/cn/tello/downloads

    # 控制命令
    def command(self):
        """进入SDK命令模式"""
        self.send_command('command')

    def takeoff(self):
        """自动起飞，1米左右"""
        self.send_command('takeoff')

    def land(self):
        """自动降落"""
        self.send_command('land')

    def streamon(self):
        """打开视频流"""
        self.send_command('streamon')
        self.stream_state = True
        self.video_thread = threading.Thread(target=self._video_thread)
        self.video_thread.daemon = True
        self.video_thread.start()

    def streamoff(self):
        """关闭视频流"""
        self.stream_state = False
        self.send_command('streamoff')

    def emergency(self):
        """停止电机转动"""
        self.send_command('emergency')

    def up(self, x: int):
        """向上飞x（20-500）厘米"""
        self.send_command('up {}'.format(x))

    def down(self, x: int):
        """向下飞x（20-500）厘米"""
        self.send_command('down {}'.format(x))

    def left(self, x: int):
        """向左飞x（20-500）厘米"""
        self.send_command('left {}'.format(x))

    def right(self, x: int):
        """向右飞x（20-500）厘米"""
        self.send_command('right {}'.format(x))

    def forward(self, x: int):
        """向前飞x（20-500）厘米"""
        self.send_command('forward {}'.format(x))

    def back(self, x: int):
        """向后飞x（20-500）厘米"""
        self.send_command('back {}'.format(x))

    def cw(self, angle: int):
        """顺时针旋转angle°（1-360）"""
        self.send_command('cw {}'.format(angle))

    def ccw(self, angle: int):
        """逆时针旋转angle°（1-360）"""
        self.send_command('ccw {}'.format(angle))

    def flip(self, direction: str):
        """朝direction方向翻滚，左侧（left）缩写为l，同理right=r，forward=f，back=b"""
        self.send_command('flip {}'.format(direction))

    def go(self, x: int, y: int, z: int, speed: int):
        """以设置速度speed（cm / s）飞往坐标（x, y, z）
            x: -500 - 500
            y: -500 - 500
            z: -500 - 500
            speed: 10 - 100(cm / s)
        x、y、z不能同时在 -20 ~ 20 之间"""
        self.send_command('go {} {} {} {}'.format(x, y, z, speed))

    def stop(self):
        """"停止运动并悬停，任何时候都可以"""
        self.send_command('stop')

    def curve(self, x1: int, y1: int, z1: int, x2: int, y2: int, z2: int, speed: int):
        """以设置速度speed（ cm/s ）飞弧线，经过（x1,y1,z1）到（x2,y2,z2）
        如果弧线半径不在 0.5-10 米范围内，则返回相应提醒
            x1, x2: -500 - 500
            y1, y2: -500 - 500
            z1, z2: -500 - 500
            speed: 10-60
        x、y、z 不能同时在 -20 ~ 20 之间"""
        self.send_command('curve {} {} {} {} {} {} {}'.format(x1, y1, z1, x2, y2, z2, speed))

    def go_mid(self, x: int, y: int, z: int, speed: int, mid: str):
        """以设置速度speed（m/s）飞往设置 id 的挑战卡坐标系的（x,y,z）坐标点
            mid:
                m1/m2/~/m8：对应挑战卡上的挑战卡ID
                m-1: 无人机内部算法最快识别到的挑战卡，随机选择一个探测到的挑战卡
                m-2: 距离无人机中心距离最近的挑战卡
            x: -500 - 500
            y: -500 - 500
            z: 0 - 500
            speed: 10-100 (cm/s)
        x、y、z 不能同时在 -20 ~ 20 之间"""
        self.send_command('go {} {} {} {} {}'.format(x, y, z, speed, mid))

    def curve_mid(self, x1: int, y1: int, z1: int, x2: int, y2: int, z2: int, speed: int, mid: str):
        """以设置速度speed（ cm/s ）飞弧线，经过设置 mid 的挑战卡坐标系中的（x1,y1,z1）点到（x2,y2,z2）点
        如果弧线半径不在 0.5-10 米范围内，则返回相应提醒
            x1, x2: -500 - 500
            y1, y2: -500 - 500
            z1, z2: 0 - 500
            speed: 10-60
        x、y、z 不能同时在 -20 ~ 20 之间"""
        self.send_command('curve {} {} {} {} {} {} {} {}'.format(x1, y1, z1, x2, y2, z2, speed, mid))

    def jump_mid(self, x: int, y: int, z: int, speed: int, yaw: int, mid1: str, mid2: str):
        """飞往 mid1 坐标系的（x,y,z）点后悬停，识别 mid2 的挑战卡
        并在 mid2 坐标系下 (0,0,z) 的位置并旋转向到设置的 偏航yaw 值，( z>0 )"""
        self.send_command('jump {} {} {} {} {} {} {}'.format(x, y, z, speed, yaw, mid1, mid2))

    # 设置命令
    def set_speed(self, speed: int):
        """将当前速度设为 speed cm/s，speed = 10-100"""
        self.send_command('speed {}'.format(speed))

    def rc_control(self, a: int, b: int, c: int, d: int):
        """设置遥控器的 4 个通道杆量
            a: 横滚 (-100~100)
            b: 俯仰 (-100~100)
            c: 油门 (-100~100)
            d: 偏航 (-100~100)
        """
        self.send_command('rc {} {} {} {}'.format(a, b, c, d))

    def set_wifi(self, ssid: str, passwrd: str):
        """更改 无人机 Wi-Fi 密码
            ssid: 更改后的 Wi-Fi 账号
            passwrd: 更改后的 Wi-Fi 密码
        """
        self.send_command('wifi {} {}'.format(ssid, passwrd))

    def mon(self):
        """"打开挑战卡探测，默认同时打开前视和下视探测"""
        self.send_command('mon')

    def moff(self):
        """"关闭挑战卡探测"""
        self.send_command('moff')

    def mdirection(self, mdir: int):
        """mdir=0/1/2
            0 打开下视探测
            1 打开前视探测
            2 同时打开前视和下视探测
        * 使用前必须使用 mon 命令打开探测功能
        * 单独打开前视或者下视探测时，探测频率为20Hz，同时打开前视和下视时，将交替探测，单个反向的探测频率为 10Hz"""
        self.send_command('mdirection {}'.format(mdir))

    def ap2sta(self, ssid: str, passwrd: str):
        """将Tello转为 station 模式，并连入到 AP
            ssid: 要连接的 Wi-Fi 账号
            passwrd: 要连接的 Wi-Fi 密码"""
        self.send_command('ap {} {}'.format(ssid, passwrd))

    # 读取命令
    def get_speed(self):
        """获取当前设置速度speed（cm/s），speed(10-100)"""
        self.send_command('speed?', True)
        return self.log[-1].get_response()

    def get_battery(self):
        """获取当前电池剩余电量的百分比值 x，x = (10-100)"""
        self.send_command('battery?', True)
        return self.log[-1].get_response()

    def get_time(self):
        """获取电机运转时间（s）"""
        self.send_command('time?', True)
        return self.log[-1].get_response()

    def get_wifi(self):
        """获得 Wi-Fi 信噪比"""
        self.send_command('wifi?', True)
        return self.log[-1].get_response()

    def get_sdk(self):
        """获得 无人机 SDK 版本号 xx(>=20)"""
        self.send_command('sdk?', True)
        return self.log[-1].get_response()

    def get_sn(self):
        """获得 无人机 SN 码 生产序列号"""
        self.send_command('sn?', True)
        return self.log[-1].get_response()

    def get_height(self):
        """获取高度，新版本中已停用"""
        self.send_command('height?', True)
        return self.log[-1].get_response()

    def get_temp(self):
        """获取温度，新版本中已停用"""
        self.send_command('temp?', True)
        return self.log[-1].get_response()

    def get_attitude(self):
        """获取飞行姿态，新版本中已停用"""
        self.send_command('attitude?', True)
        return self.log[-1].get_response()

    def get_baro(self):
        """获取压力，新版本中已停用"""
        self.send_command('baro?', True)
        return self.log[-1].get_response()

    def get_acceleration(self):
        """获取加速度，新版本中已停用"""
        self.send_command('acceleration?', True)
        return self.log[-1].get_response()

    def get_tof(self):
        """获取飞行时间，新版本中已停用"""
        self.send_command('tof?', True)
        return self.log[-1].get_response()
