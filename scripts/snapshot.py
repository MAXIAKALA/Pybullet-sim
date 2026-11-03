import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import cv2, numpy as np
from wireconnect.scene import WireConnectScene, SceneConfig
from wireconnect.perception import ColorFiducialDetector
from wireconnect.utils import camera_intrinsics

scene = WireConnectScene(SceneConfig())
scene.reset()
rgb, _ = scene.render()
cv2.imwrite("media/scene.jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])
K = camera_intrinsics(scene.cfg.cam_width, scene.cfg.cam_height, scene.cfg.cam_vfov)
img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR).copy()
for colors, tag in [(("red", "green", "blue", "yellow"), "PLUG"),
                    (("magenta", "cyan", "orange", "purple"), "PANEL")]:
    det = ColorFiducialDetector(K, scene.cfg.marker_half, colors).detect(rgb)
    if det is None:
        continue
    for name, (x, y) in det["centroids"].items():
        cv2.circle(img, (int(x), int(y)), 7, (255, 255, 255), 2)
        cv2.circle(img, (int(x), int(y)), 7, (20, 20, 20), 1)
    cx = int(np.mean([p[0] for p in det["centroids"].values()]))
    cy = int(np.mean([p[1] for p in det["centroids"].values()]))
    cv2.putText(img, tag, (cx - 22, cy + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(img, tag, (cx - 22, cy + 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
cv2.putText(img, "4-colour fiducial -> solvePnP", (14, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (20, 20, 20), 2, cv2.LINE_AA)
cv2.putText(img, "4-colour fiducial -> solvePnP", (14, 28), cv2.FONT_HERSHEY_SIMPLEX,
            0.55, (255, 255, 255), 1, cv2.LINE_AA)
cv2.imwrite("media/perception.jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 95])
print("saved media/scene.jpg, media/perception.jpg")
scene.close()
