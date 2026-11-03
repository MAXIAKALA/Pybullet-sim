import sys, os, glob, subprocess
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import cv2, numpy as np
from wireconnect.scene import WireConnectScene, SceneConfig
from wireconnect.robot import PandaRobot
from wireconnect.task import WireConnectTask

MEDIA = os.path.join(os.path.dirname(__file__), "..", "media")
FRAMES = os.path.join(MEDIA, "frames")
os.makedirs(FRAMES, exist_ok=True)
for f in glob.glob(os.path.join(FRAMES, "*.jpg")):
    os.remove(f)

scene = WireConnectScene(SceneConfig())
task = WireConnectTask(scene, verbose=True)
rec = {"i": 0, "n": 0}

def overlay(img, state, force, lat, success=False):
    h, w = img.shape[:2]
    bar = img.copy()
    cv2.rectangle(bar, (0, 0), (w, 54), (20, 22, 26), -1)
    cv2.addWeighted(bar, 0.72, img, 0.28, 0, img)
    cv2.putText(img, "WireConnect-F  |  vision-guided connector insertion", (14, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (235, 238, 242), 1, cv2.LINE_AA)
    cv2.putText(img, f"STATE {state:<8s}   F {force:5.1f} N   LAT {lat*1000:5.1f} mm",
                (14, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (150, 200, 255), 1, cv2.LINE_AA)
    if success:
        cv2.rectangle(img, (0, h - 46), (w, h), (30, 120, 60), -1)
        cv2.putText(img, "INSERTION VERIFIED  -  plug seated in socket", (14, h - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)

def on_step():
    scene.sync()
    rec["n"] += 1
    if rec["n"] % 8:
        return
    rgb, _ = scene.render()
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    overlay(img, task.state, task.last_force, task.last_lateral, task.state == "VERIFY")
    cv2.imwrite(os.path.join(FRAMES, f"f{rec['i']:05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    rec["i"] += 1

task.robot.on_step = on_step
res = task.run(randomize=False)
print(res)
rgb, _ = scene.render()
img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
overlay(img, task.state, task.last_force, task.last_lateral, res.success)
cv2.imwrite(os.path.join(MEDIA, "demo_still.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 95])

# final close-up: plug seated, arm parked, marker hidden
scene.hide_plug_marker()
for _ in range(75):
    rgb_side = scene.render_side(640, 480)
    img = cv2.cvtColor(rgb_side, cv2.COLOR_RGB2BGR)
    overlay(img, "VERIFIED", 0.0, task.last_lateral, True)
    cv2.putText(img, "SOCKET CLOSE-UP", (14, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (235, 238, 242), 1, cv2.LINE_AA)
    cv2.imwrite(os.path.join(FRAMES, f"f{rec['i']:05d}.jpg"), img, [cv2.IMWRITE_JPEG_QUALITY, 92])
    rec["i"] += 1

cmd = ["ffmpeg", "-y", "-framerate", "30", "-i", os.path.join(FRAMES, "f%05d.jpg"),
       "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
       os.path.join(MEDIA, "demo.mp4")]
subprocess.run(cmd, check=True, capture_output=True)
print("video:", os.path.join(MEDIA, "demo.mp4"), "frames", rec["i"])
scene.close()
