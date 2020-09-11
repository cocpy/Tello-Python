from tello_python import tello


drone = tello.Tello()

# 起飞
drone.takeoff()

# 前进100cm
drone.forward(100)

# 旋转90°
drone.cw(90)

# 左翻滚
drone.flip('l')

# 打开视频流
drone.streamon()

# 降落
drone.land()
