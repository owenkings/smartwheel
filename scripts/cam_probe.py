import cv2, time

def fourcc_str(cap):
    v = int(cap.get(cv2.CAP_PROP_FOURCC))
    return "".join([chr((v >> (8*i)) & 0xFF) for i in range(4)])

def probe(idx, mjpg=True):
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    if not cap.isOpened():
        return f"video{idx}: not openable"
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    ok, frame = cap.read()
    shape = None if frame is None else frame.shape
    res = f"video{idx}: open ok={ok} shape={shape} fourcc={fourcc_str(cap)}"
    cap.release()
    return res

print("=== individual capture probe (even=expected capture, odd=metadata) ===")
for i in range(8):
    print(probe(i))

print("\n=== simultaneous open of 4 capture nodes (0,2,4,6) MJPG 640x480 ===")
caps = []
for i in (0, 2, 4, 6):
    c = cv2.VideoCapture(i, cv2.CAP_V4L2)
    c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    c.set(cv2.CAP_PROP_FRAME_WIDTH, 640); c.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    caps.append((i, c))
time.sleep(0.5)
for i, c in caps:
    ok, f = c.read()
    print(f"video{i}: ok={ok} shape={None if f is None else f.shape}")
for _, c in caps:
    c.release()
