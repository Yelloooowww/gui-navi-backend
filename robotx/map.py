#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from matplotlib import pyplot as plt
import numpy as np
import pandas as pd


def rotate(degree, x, y):
  theta = np.radians(degree)
  c, s = np.cos(theta), np.sin(theta)
  R = np.array(((c, -s), (s, c)))
  P = np.array(((x, y)))
  Pr = np.matmul(R,P)
  return Pr[0], Pr[1]

class BH_Cave:
	def __init__(self):
		fig, ax = plt.subplots()
		im = plt.imread('cave_map_3.png')

		pt_bl = (-73, -19.25) # bottom left corner in meters
		pt_tr = (2, 16) # top left corner in meters
		scale_x = im.shape[0] / (pt_tr[1] - pt_bl[1] + 1) # pixels / meter
		scale_y = im.shape[1] / (pt_tr[0] - pt_bl[0] + 1) # pixels / meter
		print(scale_x, scale_y)

		ax.imshow(im, aspect=1, extent=(-73, 2, -19.25, 16))

		# anchor position is in meters
		anchor = {'loc_a1' : [-28.685,34.656], #18
		          'loc_a2' : [-35.433,34.664], #4
		          'loc_a3' : [-49.918,33.204], #5
		          'loc_a4' : [-39.195,29.093], #11
		          'loc_a5' : [-29.319,21.448], #19
		          'loc_a6' : [-16.638,14.467], #24
		          'loc_a7' : [-7.777,6.896], #12
		          'loc_a8' : [-9.006,-2.558]} #20

		anchor_val = list(anchor.values())
		anchor_location  = pd.DataFrame(data=anchor)
		anchor_location.head()

		count = 1
		for i in anchor_val:
		  a_x, a_y = rotate(32, i[0], i[1])
		  plt.scatter(a_x,a_y-1, s=150, c = 'r', marker='^', zorder=3)
		  plt.text(a_x-2, a_y, str(count), fontsize=14, c = 'r')
		  count+=1

		cid = fig.canvas.mpl_connect('button_press_event', self.onclick)
		plt.show()
		
	def onclick(self,event):
		if event.dblclick:
			print(event.x, event.y, event.xdata, event.ydata)


if __name__ == '__main__':
	BH_Cave()
