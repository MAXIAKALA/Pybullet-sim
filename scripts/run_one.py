import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import numpy as np
from wireconnect.scene import WireConnectScene, SceneConfig
from wireconnect.task import WireConnectTask
scene = WireConnectScene(SceneConfig())
task = WireConnectTask(scene)
res = task.run(randomize=False)
print(res)
scene.close()
