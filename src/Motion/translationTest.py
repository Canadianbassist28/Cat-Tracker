
import pyrealsense2 as rs
import matplotlib.pyplot as plt
import numpy as np
import cv2
from realsenseMotion import realsenseMotion
from time import clock as timer
timer() 


pipeline = rs.pipeline()
cfg = rs.config()
#cfg.enable_stream(rs.stream.accel)
#cfg.enable_stream(rs.stream.gyro)
#cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 60)
#cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60) #reads form sensor
cfg.enable_device_from_file("sample.bag", True)

profile = pipeline.start(cfg)\

plot = []
XplotData = []
YplotData = []

motion = realsenseMotion(runCalibration = True)
print("starting stream...")

colorizer = rs.colorizer(3)
align_to = rs.stream.color
align = rs.align(align_to)
thresholdFilter = rs.threshold_filter(2.5,5.0)
spatialFilter= rs.spatial_filter(.25, 50, 5, 1)
disparity = rs.disparity_transform(True)
kernel = np.ones((3,3),np.uint8)
try:
    while 1:

        start = timer()
        try:
            frames = pipeline.wait_for_frames()
            frames = align.process(frames)
            timeStamp = frames.get_timestamp() / 1000
        except:
            break
        motion.get_data(frames, timeStamp)


        depthFrame = frames.get_depth_frame()
        depthFrame1 = thresholdFilter.process(depthFrame)
        depthFrame1 = disparity.process(depthFrame1)
        depthFrame1 = spatialFilter.process(depthFrame1)
        depthFrame1 = colorizer.colorize(depthFrame1)

        colorImg = np.asanyarray(frames.get_color_frame().get_data())
        tmp = np.asanyarray(depthFrame1.get_data())

        depthFrame = colorizer.colorize(depthFrame)
        depthFrame = np.asanyarray(depthFrame.get_data())

        low = np.array([120, 120, 120])
        high = np.array([150, 150, 150])
        mask = cv2.inRange(tmp, low, high)
        tmp = cv2.GaussianBlur(tmp,(7,7),1)

        cnts,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(colorImg, cnts,-1, [0,255,0], 2, cv2.LINE_AA)

        mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        colorImg = cv2.cvtColor(colorImg, cv2.COLOR_RGB2BGR)
        
        FPS = "{:.2f}".format(1 / (timer() - start))
        cv2.putText(colorImg, FPS, (10,15), cv2.FONT_HERSHEY_SIMPLEX, .3, (0,0,0), 1, cv2.LINE_AA)

        cv2.namedWindow('RealSenseSpatial', cv2.WINDOW_AUTOSIZE)
        res = np.hstack((mask, colorImg))
        cv2.imshow('RealSenseSpatial', res)
        
        plot.append(motion.angle)
        #XplotData.append(motion.position[0])
        #YplotData.append(motion.position[2])


        if (cv2.waitKey(1) & 0xFF == ord('q')):
        	break;
finally:
    pipeline.stop()

cv2.destroyAllWindows()
print("Plot Result")
plt.grid()

plt.plot(plot)
#plt.plot(XplotData, YplotData)
#plt.plot(YplotData)
plt.show()
