# 智能送药小车 —— OpenMV 端数字识别
#
# 配合 STM32F103RCT6 主控（USART3，115200）：
#   STM32 -> OpenMV : b"*<Find_Task><Target_Num>&"   约每 21ms 一帧，4 字节
#   OpenMV -> STM32 : [0x2C, 0x12, Num, LoR, find_flag, Find_Task, 0x5B]   7 字节
#
#   Find_Task = 1 : 药房第一次识别，只看画面中间（ROI 55,0,50,40）
#   Find_Task = 2 : 行进中识别，看整幅（ROI 0,0,160,40），按匹配框位置判断数字偏左还是偏右
#
# 模板（全部放在 OpenMV 存储根目录，命名必须一致）：
#   /1.pgm ~ /8.pgm                              模式 1 用，每个数字一张
#   /3L.pgm /3LL.pgm /3R.pgm /3RR.pgm … /8RR.pgm  模式 2 用，每个数字四个角度
#
# 本文件相对上游 FindNumber.py 只做了四件事（阈值 0.8、ROI、step、search 全部未改）：
#   1. 启用了 2~8 的模板加载与匹配
#   2. 启用了 Find_Task == 2（行进中识别）
#   3. LoR 判断补全（原来 R[0] == 0 时 LoR 未赋值会抛 NameError）
#   4. 每帧先清 find_flag；没识别到时也发一帧，让 STM32 能清掉上一次的旧值

import time, sensor, image
from image import SEARCH_EX
from pyb import UART, LED

uart = UART(3, 115200, timeout_char = 1000)

sensor.reset()
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.QQVGA)            # 160x120
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_windowing(0, 40, 160, 40)          # 观察窗：只取中间 160x40 的一条

# ---------------- 加载模板 ----------------
# 模式 1：每个数字一张
template01 = image.Image("/1.pgm")
template02 = image.Image("/2.pgm")
template03 = image.Image("/3.pgm")
template04 = image.Image("/4.pgm")
template05 = image.Image("/5.pgm")
template06 = image.Image("/6.pgm")
template07 = image.Image("/7.pgm")
template08 = image.Image("/8.pgm")

# 模式 2：每个数字四个角度
template3L  = image.Image("/3L.pgm")
template3LL = image.Image("/3LL.pgm")
template3R  = image.Image("/3R.pgm")
template3RR = image.Image("/3RR.pgm")

template4L  = image.Image("/4L.pgm")
template4LL = image.Image("/4LL.pgm")
template4R  = image.Image("/4R.pgm")
template4RR = image.Image("/4RR.pgm")

template5L  = image.Image("/5L.pgm")
template5LL = image.Image("/5LL.pgm")
template5R  = image.Image("/5R.pgm")
template5RR = image.Image("/5RR.pgm")

template6L  = image.Image("/6L.pgm")
template6LL = image.Image("/6LL.pgm")
template6R  = image.Image("/6R.pgm")
template6RR = image.Image("/6RR.pgm")

template7L  = image.Image("/7L.pgm")
template7LL = image.Image("/7LL.pgm")
template7R  = image.Image("/7R.pgm")
template7RR = image.Image("/7RR.pgm")

template8L  = image.Image("/8L.pgm")
template8LL = image.Image("/8LL.pgm")
template8R  = image.Image("/8R.pgm")
template8RR = image.Image("/8RR.pgm")

# 模式 1：按 1~8 的顺序找，第一个命中的就发出去
MODE1_TEMPLATES = (
    (1, template01), (2, template02), (3, template03), (4, template04),
    (5, template05), (6, template06), (7, template07), (8, template08),
)

# 模式 2：数字 -> 它的四个角度模板
MODE2_TEMPLATES = {
    3: (template3L, template3LL, template3R, template3RR),
    4: (template4L, template4LL, template4R, template4RR),
    5: (template5L, template5LL, template5R, template5RR),
    6: (template6L, template6LL, template6R, template6RR),
    7: (template7L, template7LL, template7R, template7RR),
    8: (template8L, template8LL, template8R, template8RR),
}

# ---------------- 串口 ----------------
# 通过接收 STM32 传来的数据决定用哪种模式
Find_Task = 1
Target_Num = 0
find_flag = 0
x = 0
data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

blue_led = LED(3)          # 板上蓝灯，调试用


def UartReceiveDate():
    # 这个函数不能运行太快，否则串口读取会出错
    global Find_Task
    global Target_Num
    global x
    global data

    data[0] = uart.readchar()
    data[1] = uart.readchar()
    data[2] = uart.readchar()
    data[3] = uart.readchar()
    data[4] = uart.readchar()
    data[5] = uart.readchar()
    data[6] = uart.readchar()
    data[7] = uart.readchar()

    if data[x] == 42 and data[x + 3] == 38 and x < 5:      # '*' ... '&'
        Find_Task = data[x + 1] - 48
        Target_Num = data[x + 2] - 48
        print(Find_Task, Target_Num)
        x = 0
    elif x >= 5:
        x = 0
    x += 1


def SendResult(num, lor, flag):
    # 一帧 7 字节：帧头 0x2C 0x12，数据，帧尾 0x5B
    uart.write(bytearray([0x2C, 0x12, num, lor, flag, Find_Task, 0x5B]))


# ---------------- 模板匹配 ----------------
def FirstFindTemplate(template):
    # 模式 1：只检测中间那一块
    R = img.find_template(template, 0.8, step=1, roi=(55, 0, 50, 40), search=SEARCH_EX)
    return R


def FirstFindedNum(R, Finded_Num):
    global find_flag
    img.draw_rectangle(R, color=(225, 0, 0))
    find_flag = 1
    # 药房这次识别不需要左右，LoR 固定 0
    SendResult(Finded_Num, 0, 1)
    print("目标病房号：", Finded_Num)


def FindTemplate(template):
    # 模式 2：整幅搜索
    R = img.find_template(template, 0.8, step=1, roi=(0, 0, 160, 40), search=SEARCH_EX)
    return R


def FindedNum(R, Finded_Num):
    global find_flag
    img.draw_rectangle(R, color=(225, 0, 0))
    # 返回值是匹配框的左边缘：ROI 宽 160，中值 80，框边缘再往里取 65
    if R[0] >= 65:
        LoR = 2        # 2 是右
    else:
        LoR = 1        # 1 是左（R[0] == 0 也归到左边）
    find_flag = 1
    SendResult(Finded_Num, LoR, 1)
    print("识别到的数字是：", Finded_Num, "此数字所在方位：", LoR)


clock = time.clock()

while (True):
    clock.tick()
    img = sensor.snapshot()

    UartReceiveDate()

    # 每帧先当作"没识别到"，识别到再置 1，避免 STM32 一直用上一次的旧值
    find_flag = 0

    if Find_Task == 1:
        # 药房第一次识别：1~8 依次匹配，第一个命中的发出去
        for num, tpl in MODE1_TEMPLATES:
            r = FirstFindTemplate(tpl)
            if r:
                FirstFindedNum(r, num)
                break
        else:
            SendResult(0, 0, 0)          # 都没匹配上

    elif Find_Task == 2:
        # 行进中识别：在目标数字的几个角度模板里找一个
        hit = None
        for tpl in MODE2_TEMPLATES.get(Target_Num, ()):
            r = FindTemplate(tpl)
            if r:
                hit = r
                break
        if hit:
            FindedNum(hit, Target_Num)
        else:
            SendResult(0, 0, 0)

    print(clock.fps(), Find_Task, Target_Num)
