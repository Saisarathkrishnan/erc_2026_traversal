#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import MarkerTag
import numpy as np
import cv2
import tensorrt as trt
import pycuda.driver as cuda
import pycuda.autoinit
from flask import Flask, Response
import threading
import time

app = Flask(__name__)
latest_frame = None
frame_lock = threading.Lock()

@app.route("/video")
def video():
    def gen():
        global latest_frame
        while True:
            with frame_lock:
                frame = None if latest_frame is None else latest_frame.copy()
            if frame is None:
                time.sleep(0.01)
                continue
            ok, jpg = cv2.imencode(".jpg", frame)
            if not ok:
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n"
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

def start_flask():
    app.run(host="0.0.0.0", port=5001, debug=False, use_reloader=False, threaded=True)

class TRTInfer:
    def __init__(self, engine_path):
        logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, "rb") as f, trt.Runtime(logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()
        self.input_name = self.engine.get_tensor_name(0)
        self.output_name = self.engine.get_tensor_name(1)
        self.input_shape = self.engine.get_tensor_shape(self.input_name)
        self.output_shape = self.engine.get_tensor_shape(self.output_name)
        self.d_input = cuda.mem_alloc(trt.volume(self.input_shape) * np.float32().nbytes)
        self.d_output = cuda.mem_alloc(trt.volume(self.output_shape) * np.float32().nbytes)
        self.context.set_tensor_address(self.input_name, int(self.d_input))
        self.context.set_tensor_address(self.output_name, int(self.d_output))
        self.h_output = np.empty(self.output_shape, dtype=np.float32)

    def infer(self, img):
        img = np.ascontiguousarray(img, dtype=np.float32)
        cuda.memcpy_htod_async(self.d_input, img, self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.h_output, self.d_output, self.stream)
        self.stream.synchronize()
        return self.h_output

class InferenceEngine(Node):
    def __init__(self):
        super().__init__("inference_engine")
        self.bridge = CvBridge()
        self.trt = TRTInfer("/home/mrmnavjet/IRC2026/ircWS/src/yolo/yolo/cone_v1.engine")
        self.latest_rgb = None
        self.latest_depth = None
        self.COLOR_ID = {"orange":1,"red":2,"blue":3,"green":4,"yellow":5}

        self.fps_time = time.time()
        self.fps_count = 0
        self.fps = 0.0

        self.sub_rgb = self.create_subscription(Image,"/zed/zed_node/rgb/color/rect/image",self.rgb_cb,10)
        self.sub_depth = self.create_subscription(Image,"/zed/zed_node/depth/depth_registered",self.depth_cb,10)
        self.pub = self.create_publisher(MarkerTag,"/marker_detect",10)
        self.timer = self.create_timer(0.033,self.process)

        threading.Thread(target=start_flask,daemon=True).start()

    def rgb_cb(self,msg): self.latest_rgb = msg
    def depth_cb(self,msg): self.latest_depth = msg

    def calculate_iou(self,a,b):
        x1=max(a[0],b[0]); y1=max(a[1],b[1])
        x2=min(a[2],b[2]); y2=min(a[3],b[3])
        inter=max(0,x2-x1)*max(0,y2-y1)
        area1=(a[2]-a[0])*(a[3]-a[1])
        area2=(b[2]-b[0])*(b[3]-b[1])
        return inter/(area1+area2-inter+1e-6)

    def nms(self,dets,t=0.5):
        dets=sorted(dets,key=lambda x:x[4],reverse=True)
        keep=[]
        while dets:
            best=dets.pop(0)
            keep.append(best)
            dets=[d for d in dets if self.calculate_iou(best,d)<t]
        return keep

    def calculate_distance(self,depth,cx,cy):
        h,w=depth.shape
        r=8
        p=depth[max(0,cy-r):min(h,cy+r),max(0,cx-r):min(w,cx+r)].astype(np.float32)
        p=p[(p>0.2)&(p<5.0)]
        if p.size<20: return None
        m=np.median(p)
        mad=np.median(np.abs(p-m))
        if mad>0.08: return None
        i=p[np.abs(p-m)<0.05]
        if i.size<10: return None
        return float(np.mean(i))

    def detect_color(self,roi):
        hsv=cv2.cvtColor(roi,cv2.COLOR_BGR2HSV)
        ranges={"orange":((5,120,120),(20,255,255)),"red":((0,140,120),(10,255,255)),
                "blue":((90,120,120),(130,255,255)),"green":((40,120,120),(80,255,255)),
                "yellow":((22,120,120),(34,255,255))}
        best=None;mx=0
        for c,(lo,hi) in ranges.items():
            cnt=cv2.countNonZero(cv2.inRange(hsv,lo,hi))
            if cnt>mx: best, mx=c,cnt
        return best

    def get_draw_color(self,name):
        return {"orange":(0,140,255),"red":(0,0,255),"blue":(255,0,0),
                "green":(0,255,0),"yellow":(0,255,255)}[name]

    def process(self):
        global latest_frame

        self.fps_count+=1
        if time.time()-self.fps_time>1:
            self.fps=self.fps_count/(time.time()-self.fps_time)
            self.fps_time=time.time()
            self.fps_count=0

        if self.latest_rgb is None or self.latest_depth is None: return

        frame=self.bridge.imgmsg_to_cv2(self.latest_rgb,"bgr8")
        depth=self.bridge.imgmsg_to_cv2(self.latest_depth)
        if depth.dtype==np.uint16: depth=depth.astype(np.float32)*0.001

        img=cv2.resize(frame,(640,640))
        img=img.transpose(2,0,1)[None]/255.0
        out=self.trt.infer(img)[0]

        h,w,_=frame.shape
        sx,sy=w/640,h/640
        raw=[]

        for d in out.T:
            if d[4]<0.70: continue
            cx,cy=int(d[0]*sx),int(d[1]*sy)
            bw,bh=int(d[2]*sx),int(d[3]*sy)
            x1,y1,x2,y2=max(0,cx-bw//2),max(0,cy-bh//2),min(w,cx+bw//2),min(h,cy+bh//2)
            raw.append((x1,y1,x2,y2,d[4],cx,cy))

        dets=self.nms(raw,0.5)

        for x1,y1,x2,y2,conf,cx,cy in dets:
            roi=frame[y1:y2,x1:x2]
            if roi.size==0: continue
            color=self.detect_color(roi)
            if color is None: continue

            dist=self.calculate_distance(depth,cx,cy)
            if dist is None: continue

            msg=MarkerTag()
            msg.is_found=True
            msg.id=self.COLOR_ID[color]
            msg.x=dist
            msg.y=-(cx-w/2)/558.0
            self.pub.publish(msg)

            draw=self.get_draw_color(color)
            cv2.rectangle(frame,(x1,y1),(x2,y2),draw,2)
            cv2.putText(frame,f"{color} {dist:.2f}m",(x1,y1-6),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,draw,2)

        cv2.putText(frame,f"FPS: {self.fps:.1f}",(10,frame.shape[0]-15),
                    cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)

        with frame_lock:
            latest_frame=frame.copy()

def main():
    rclpy.init()
    rclpy.spin(InferenceEngine())
    rclpy.shutdown()

if __name__=="__main__":
    main()
