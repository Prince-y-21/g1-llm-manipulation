from pathlib import Path
import cv2
r=Path('/home/unitree/codex_g1_tasks/pour_water_20261019');img=cv2.imread(str(r/'head_color.jpg'))
cv2.imwrite(str(r/'bottle_detail.jpg'),cv2.resize(img[205:385,315:410],(380,720)))
cv2.imwrite(str(r/'cup_detail.jpg'),cv2.resize(img[240:350,78:210],(528,440)))
