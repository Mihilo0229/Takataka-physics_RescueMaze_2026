import sensor, image, time, ustruct, gc, uos, ml
from machine import UART

# =========================
# UART 初期化
# =========================
uart = UART(1, baudrate=19200, timeout=200)

HEADER = 0xAA

# =========================
# カメラ初期化
# =========================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((120, 40, 140, 140))
sensor.skip_frames(time=2000)

# =========================
# 色しきい値
# =========================
yellow_threshold = (41, 61, -3, 29, 60, 90)
red_threshold    = (26, 54, 31, 44, 13, 41)

# =========================
# ML（必要な場合のみ）
# =========================
net = ml.Model(
    "trained.tflite",
    load_to_fb=uos.stat("trained.tflite")[6] > (gc.mem_free() - 64 * 1024)
)
labels = [line.rstrip('\n') for line in open("labels.txt")]

clock = time.clock()

cooldown = 0

# =========================
# 送信関数（★重要）
# =========================
def send_event(code):
    chk = code ^ 0xFF
    uart.write(bytes([HEADER, code, chk]))
    print("SEND:", code)

# =========================
# メインループ
# =========================
while True:
    clock.tick()
    img = sensor.snapshot()

    send_code = 0

    # ---- 赤（4） ----
    blobs = img.find_blobs([red_threshold])
    if blobs and len(blobs) >= 5:
        send_code = 4

    # ---- 黄（5） ----
    blobs2 = img.find_blobs([yellow_threshold])
    if blobs2 and len(blobs2) >= 2:
        send_code = 5

    # ---- ML（1,2）----
    predictions = net.predict([img])[0].flatten().tolist()
    for label, conf in zip(labels, predictions):
        if conf >= 0.7:
            if label == "H":
                send_code = 1
            elif label == "S":
                send_code = 2

    # ---- UART送信 ----
    if send_code != 0 and cooldown == 0:
        try:
            send_event(send_code)
            cooldown = 100
        except OSError:
            pass

    if cooldown > 0:
        cooldown -= 1

    gc.collect()
    time.sleep_ms(50)
