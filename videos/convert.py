import os
import cv2
import numpy as np

path = "/home/silveryu1999/dataset/1059/"
fps = 10
video_writer = cv2.VideoWriter(filename="./video.mp4", fourcc=cv2.VideoWriter_fourcc(*'mp4v'), fps=fps, frameSize=(1238,374))

for i in range(1059):
	if i >=0 and i<=9:
		if os.path.exists(path + '000000000' + str(i) + '.png'):
			img = cv2.imread(filename=path + '000000000' + str(i) + '.png')
			cv2.waitKey(1)
			video_writer.write(img)
	elif i>=10 and i<=99:
		if os.path.exists(path + '00000000' + str(i) + '.png'):
			img = cv2.imread(filename=path + '00000000' + str(i) + '.png')
			cv2.waitKey(1)
			video_writer.write(img)
	elif i>=100 and i<=999:
		if os.path.exists(path + '0000000' + str(i) + '.png'):
			img = cv2.imread(filename=path + '0000000' + str(i) + '.png')
			cv2.waitKey(1)
			video_writer.write(img)
	elif i>=1000 and i<=9999:
		if os.path.exists(path + '000000' + str(i) + '.png'):
			img = cv2.imread(filename=path + '000000' + str(i) + '.png')
			cv2.waitKey(1)
			video_writer.write(img)
video_writer.release()
