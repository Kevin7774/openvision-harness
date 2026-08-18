import os,sys,threading,time,numpy as np,cv2,rclpy,tf2_ros,math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, CameraInfo
sys.path.insert(0,"/home/astrabot/workspace/zed_ws/scripts")
from handeye_block import Sensors, find_block, quat_to_R, project
from grasp_block import TIP_CENTER, COLOR_RANGES, ZED_OPTICAL_FRAME
threading.Thread(target=lambda:(time.sleep(90),sys.stdout.flush(),os._exit(9)),daemon=True).start()
rclpy.init(); s=Sensors()
if not s.wait(20.0,need=("rgb","info")): print("no data"); sys.stdout.flush(); os._exit(2)
s.wait(1.5,need=("rgb",))
K=s.info; img=s.rgb.copy()
Ttcp=Topt=None
import time as _t
_t0=_t.time()
while _t.time()-_t0<15 and (Ttcp is None or Topt is None):
    rclpy.spin_once(s,timeout_sec=0.05)
    Ttcp=s.T("right_tcp_link"); Topt=s.T(ZED_OPTICAL_FRAME)
if Ttcp is None or Topt is None:
    print(f"TF 缺: tcp={Ttcp is not None} opt={Topt is not None}"); sys.stdout.flush(); os._exit(3)
Pfk=Ttcp[:3,3]+Ttcp[:3,:3]@np.asarray(TIP_CENTER,float)
Pin=(np.linalg.inv(Topt)@np.append(Pfk,1.0))[:3]; pred=project(Pin,K)
print(f"K: fx={K[0]:.1f} fy={K[1]:.1f} cx={K[2]:.1f} cy={K[3]:.1f}  半视场={math.degrees(math.atan(K[2]/K[0])):.1f}deg")
print(f"P_fk_in_opt={Pin.round(4)}  方位角={math.degrees(math.atan2(Pin[0],Pin[2])):.1f}deg  pred_uv={np.round(pred,1)}")
for name,col in (("yellow",(0,255,255)),("orange",(0,140,255)),("green",(0,255,0))):
    _,blobs,lab,_=find_block(img,name,pred,1e9)
    for b in blobs[:6]:
        u,v=int(b["u"]),int(b["v"]); ys,xs=np.where(lab==b["label"])
        cv2.rectangle(img,(xs.min(),ys.min()),(xs.max(),ys.max()),col,2)
        cv2.putText(img,f'{name[:1].upper()}{b["area"]}',(xs.min(),max(12,ys.min()-4)),0,0.5,col,1)
        print(f"  {name:6s} area={b['area']:6d} uv=({b['u']:6.1f},{b['v']:6.1f}) bbox=({xs.min()},{ys.min()})-({xs.max()},{ys.max()}) 距预测={b['dist_px']:.0f}px")
pu,pv=int(round(pred[0])),int(round(pred[1]))
cv2.line(img,(pu-40,pv),(min(pu+40,img.shape[1]-1),pv),(255,0,255),2)
cv2.line(img,(pu,pv-40),(pu,pv+40),(255,0,255),2)
cv2.putText(img,"FK pred",(max(0,pu-120),pv-14),0,0.7,(255,0,255),2)
cv2.imwrite("/home/astrabot/workspace/zed_ws/experiments/handeye/zed_annotated.png",img)
print("存 zed_annotated.png  洋红十字=FK 预测的手中积木位置")
sys.stdout.flush(); os._exit(0)
