# Tello-Python

Control DJI Tello drone with python

# Installation

    pip install tello-python

# Examples

\# 引入tello_python包，注意：这里是下划线_连接<br>
from tello_python import tello

\# 初始化无人机对象<br>
drone = tello.Tello()

\# 起飞<br>
drone.takeoff()

\# 前进100cm<br>
drone.forward(100)

\# 顺时针旋转90°<br>
drone.cw(90)

\# 向左翻滚<br>
drone.flip('l')

\# 打开视频流<br>
drone.streamon()

\# 降落<br>
drone.land()

更多命令请参考源码tello.py文件内的方法及注释