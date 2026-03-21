#doesnt work on jetson but there for git
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import pygame
import threading
import matplotlib.pyplot as plt
import os
import datetime

WIDTH, HEIGHT = 900, 900
SCALE = 100000.0

SAVE_DIR = os.path.expanduser("~/IRC2026/ircWS/gps_maps")
os.makedirs(SAVE_DIR,exist_ok=True)

COLORS = {
    'S': (0,0,0),
    'O': (255,165,0),
    'B': (0,0,255),
    'G': (0,255,0),
    'Y': (255,255,0),
    'R': (255,0,0),
    'N': (160,32,240)
}

class GPSViewer(Node):
    def __init__(self):
        super().__init__('gps_viewer')
        self.sub = self.create_subscription(NavSatFix,'/gps',self.gps_cb,10)
        self.lat=None
        self.lon=None
        self.origin_lat=None
        self.origin_lon=None
        self.points=[]
        self.start_point=None

    def gps_cb(self,msg):
        self.lat=msg.latitude
        self.lon=msg.longitude
        if self.origin_lat is None:
            self.origin_lat=self.lat
            self.origin_lon=self.lon

    def gps_to_xy(self,lat,lon):
        dx=(lon-self.origin_lon)*SCALE
        dy=(lat-self.origin_lat)*SCALE
        return dx,dy

def ros_spin(node):
    rclpy.spin(node)

def draw_rover(screen,x,y):
    pts=[(x,y-6),(x-5,y+5),(x+5,y+5)]
    pygame.draw.polygon(screen,(60,60,60),pts)

def main():
    rclpy.init()
    node=GPSViewer()
    threading.Thread(target=ros_spin,args=(node,),daemon=True).start()

    pygame.init()
    screen=pygame.display.set_mode((WIDTH,HEIGHT))
    pygame.display.set_caption("GPS Marker Tool")
    font=pygame.font.SysFont("Arial",14)

    print("\nControls:")
    print("S     -> Start point (black)")
    print("O     -> Orange marker")
    print("B     -> Blue marker")
    print("G     -> Green marker")
    print("Y     -> Yellow marker")
    print("R     -> Red marker")
    print("N     -> New point (purple)")
    print("ENTER -> Finish, save & plot")
    print("ESC   -> Exit without saving\n")

    clock=pygame.time.Clock()
    running=True
    finish=False
    startup_frames=40

    while running:
        screen.fill((255,255,255))

        for event in pygame.event.get():
            if event.type==pygame.QUIT:
                running=False
            if event.type==pygame.KEYDOWN and node.lat is not None and startup_frames==0:
                key=pygame.key.name(event.key).upper()
                if key=="ESCAPE":
                    running=False
                if key=="RETURN":
                    finish=True
                    running=False
                if key in COLORS:
                    node.points.append((node.lat,node.lon,key))
                    if key=="S":
                        node.start_point=(node.lat,node.lon)

        if node.lat is not None:
            dx,dy=node.gps_to_xy(node.lat,node.lon)
            draw_rover(screen,WIDTH//2+int(dx),HEIGHT//2-int(dy))

        for lat,lon,key in node.points:
            dx,dy=node.gps_to_xy(lat,lon)
            pygame.draw.circle(screen,COLORS[key],(WIDTH//2+int(dx),HEIGHT//2-int(dy)),5)

        if node.start_point:
            sx,sy=node.gps_to_xy(node.start_point[0],node.start_point[1])
            txt=font.render("START",True,(0,0,0))
            screen.blit(txt,(WIDTH//2+int(sx)+8,HEIGHT//2-int(sy)-8))

        if startup_frames>0:
            startup_frames-=1

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

    if finish and len(node.points)>0:
        xs,ys,cs=[],[],[]
        for lat,lon,key in node.points:
            dx,dy=node.gps_to_xy(lat,lon)
            xs.append(dx)
            ys.append(dy)
            cs.append([c/255.0 for c in COLORS[key]])

        plt.figure("GPS Map")
        plt.scatter(xs,ys,c=cs,s=60)

        if node.start_point:
            sx,sy=node.gps_to_xy(node.start_point[0],node.start_point[1])
            plt.scatter([sx],[sy],c="black",s=120)
            plt.text(sx+5,sy+5,"START")

        plt.xlim(-250,250)
        plt.ylim(-250,250)
        plt.gca().set_aspect("equal",adjustable="box")
        plt.grid(True)

        ts=datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename=os.path.join(SAVE_DIR,f"gps_map_{ts}.png")
        plt.savefig(filename,dpi=200)
        print(f"\nSaved map to {filename}\n")

        plt.show()

    rclpy.shutdown()

if __name__=="__main__":
    main()

