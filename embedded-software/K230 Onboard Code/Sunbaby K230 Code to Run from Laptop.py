"""
实验名称：在线训练-YOLO图像检测: 基于摄像头
实验平台：01Studio CanMV K230/CanMV K230 mini
说明：可以通过display="xxx"参数选择"hdmi"、"lcd3_5"(3.5寸mipi屏)或"lcd2_4"(2.4寸mipi屏)显示方式
01科技（01Studio）在线训练平台：https://ai.01studio.cc
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from media.sensor import *
import os, sys, gc
import ulab.numpy as np
import image
import time

from machine import UART
from machine import FPIOA
from machine import Pin
import ujson

# 这里为自动生成内容，自定义场景请修改为您自己的模型路径、标签名称、模型输入大小

def get_model_config(p):
    if p == 1:
        return "/sdcard/Phase_1.kmodel", {0: 'Rock', 1: 'Teletubby'}
    elif p == 2:
        return "/sdcard/solar_panel.kmodel", {0: 'Loop', 1: 'Panel'}
    else:
        raise ValueError("Unknown phase: {}".format(p))

# starting phase on boot
phase = 1
kmodel_path, labels = get_model_config(phase)

model_input_size = [224, 224]
# 显示模式，可以选择"hdmi"、"lcd3_5"(3.5寸mipi屏)和"lcd2_4"(2.4寸mipi屏)
display = "lcd3_5"

if display == "hdmi":
    display_mode = "hdmi"
    display_size = [1920, 1080]

elif display == "lcd3_5":
    display_mode = "st7701"
    display_size = [800, 480]

elif display == "lcd2_4":
    display_mode = "st7701"
    display_size = [640, 480]

rgb888p_size = [1280, 720] # [1920, 1080] or [1280, 720] or [640, 360]

# 初始化PipeLine
pl = PipeLine(
    rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode
)

if display == "lcd2_4":
    pl.create(sensor=Sensor(width=1280, height=960))  # 创建PipeLine实例，画面4:3

else:
    pl.create(sensor=Sensor(width=1920, height=1080))  # 创建PipeLine实例

display_size = pl.get_display_size()

# 初始化YOLO11实例
confidence_threshold = 0.4  # 置信度
nms_threshold = 0.45

def create_yolo():
    y = YOLO11(
        task_type="detect",
        mode="video",
        kmodel_path=kmodel_path,
        labels=labels,
        rgb888p_size=rgb888p_size,
        model_input_size=model_input_size,
        display_size=display_size,
        conf_thresh=confidence_threshold,
        nms_thresh=nms_threshold,
        max_boxes_num=50,
        debug_mode=0,
    )
    y.config_preprocess()
    return y

yolo = create_yolo()

clock = time.clock()

fpioa = FPIOA()
fpioa.set_function(3,FPIOA.UART1_TXD)
fpioa.set_function(4,FPIOA.UART1_RXD)
uart = UART(UART.UART1, 9600)

# ---- onboard LED on IO6, internal pull-down, active-high ----
fpioa.set_function(6, FPIOA.GPIO6)
led_pin = Pin(6, Pin.OUT, pull=Pin.PULL_DOWN)
led_pin.value(0)  # start off

def blink_light(times=5, on_ms=120, off_ms=120):
    print("blinking light")
    for _ in range(times):
        led_pin.value(1)
        time.sleep_ms(on_ms)
        led_pin.value(0)
        time.sleep_ms(off_ms)
    # led_pin.value(1)

# ---- listen for commands from the ESP32 (e.g. phase switch) ----
rx_buffer = b""

def switch_phase(new_phase):
    global phase, kmodel_path, labels, yolo

    new_kmodel_path, new_labels = get_model_config(new_phase)  # raises if invalid, before we tear anything down

    print("K230: switching from phase", phase, "to phase", new_phase)
    yolo.deinit()

    kmodel_path, labels = new_kmodel_path, new_labels
    phase = new_phase
    yolo = create_yolo()
    print("K230: now running phase", phase)

def handle_command(line_bytes):
    try:
        text = line_bytes.decode().strip()
    except Exception:
        return
    if not text:
        return
    try:
        cmd = ujson.loads(text)
    except Exception as e:
        print("K230: bad command '{}': {}".format(text, e))
        return

    if cmd.get("cmd") == "set_phase":
        new_phase = cmd.get("phase")
        if new_phase == phase:
            return
        try:
            switch_phase(new_phase)
        except Exception as e:
            print("K230: failed to switch to phase {}: {}".format(new_phase, e))
    elif cmd.get("cmd") == "blink_light":
        blink_light()
    else:
        print("K230: unknown command:", cmd)

def poll_uart_commands():
    global rx_buffer
    n = uart.any()
    if n:
        data = uart.read(n)
        # print("RX:", repr(data))
        rx_buffer += data
        # print("BUFFER:", repr(rx_buffer))

    # Throw away anything before the start of a JSON object.
    start = rx_buffer.find(b"{")
    if start > 0:
        #print("Discarding:", repr(rx_buffer[:start]))
        rx_buffer = rx_buffer[start:]
    elif start == -1:
        # No JSON yet.
        return

    idx = rx_buffer.find(b"\n")
    while idx >= 0:
        line_bytes = rx_buffer[:idx]
        rx_buffer = rx_buffer[idx + 1:]
        handle_command(line_bytes)

        start = rx_buffer.find(b"{")
        if start > 0:
            rx_buffer = rx_buffer[start:]

        idx = rx_buffer.find(b"\n")

if uart.any():
    uart.read(uart.any())

# loop code
while True:

    clock.tick()

    # check for a phase-change (or other) command from the ESP32
    poll_uart_commands()

    # 逐帧推理
    img = pl.get_frame()
    res = yolo.run(img)
    yolo.draw_result(res, pl.osd_img)
    pl.show_image()

    detections = []

    boxes, class_ids, scores = res
    for box, class_id, score in zip(boxes, class_ids, scores):
        x, y, w, h = box
        # print(f"class_id={class_id}, label={labels[class_id]}, x={x}, y={y}, w={w}, h={h}, score={score:.2f}")
        x1, x2, y1, y2 = x, x+w, y, y+h
        detections.append({
                "label": labels[class_id],
                "class_id": int(class_id),
                "x1": int(x1), "y1": int(y1),
                "x2": int(x2), "y2": int(y2),
                "score": round(float(score), 3)
        })

    payload = ujson.dumps(detections) + "\n"
    uart.write(payload)

    #print(res)  # 打印识别结果
    gc.collect()
    print("FPS:", clock.fps())  # 打印帧率

# 释放资源
yolo.deinit()
pl.destroy()
