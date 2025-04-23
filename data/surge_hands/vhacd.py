import pybullet as p
import pybullet_data as pd
import os

import pybullet_data

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
name_in =  "finger_distal.obj"
name_out = "index_finger_distal_vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log)