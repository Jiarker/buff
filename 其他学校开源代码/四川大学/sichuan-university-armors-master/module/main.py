# ----------------------------------------------
#
#             SCURM_Vision_Main
#                By Pikachu
#                  主程序
#
#           LAST_UPDATE:OCT30/2020
#
# ----------------------------------------------
import cv2
import time
from ModuleCamera import Camera

temp = Camera(in_nums=0)
temp.getf()
nowt = time.time()
last = time.time()
rfps = 0
pfps = 0
while (cv2.waitKey(1) & 0xFF) != ord('q'):
    rfps = rfps + 1
    nowt = time.time()
    if nowt - last >= 1:
        last = nowt
        # print(rfps)
        pfps = rfps
        rfps = 0
    tmp = cv2.putText(temp.getp(), str(pfps), (0, 50), cv2.FONT_HERSHEY_COMPLEX, 2.0, (100, 200, 200), 5)
    cv2.imshow("Press q to end", tmp)
temp.coff()
