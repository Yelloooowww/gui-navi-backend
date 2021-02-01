#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import pandas as pd
import rospy
import requests
import json
from geometry_msgs.msg import PoseStamped,Twist,PoseWithCovariance
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest, GetPlanResponse
from visualization_msgs.msg import Marker
import numpy as np
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import roslib
import struct
import time
import rospkg
from nav_msgs.msg import Odometry,OccupancyGrid
from visualization_msgs.msg import MarkerArray,Marker
from geometry_msgs.msg import Twist



class AnchorWaypoint:
	def __init__(self,mid=1,globalx=None,globaly=None,isanchor=True,uwbid=None):
		# self.aid  #INT
		self.mid = mid #INT
		self.globalx = globalx #DECIMAL(11,9)
		self.globaly = globaly #DECIMAL(12,9)
		self.isanchor = isanchor #BOOL
		self.uwbid = uwbid
		self.post()

	def post(self):
		payload = 'mid='+str(self.mid)+'&globalx='+str(self.globalx)+'&globaly='+str(self.globaly)+'&uwbid='+str(self.uwbid)
		url = "http://robotx.cowbon.info/anchorwaypoints"
		headers = {'Content-Type': 'application/x-www-form-urlencoded'}
		response = requests.request("POST", url, headers=headers, data = payload)
		tmp = response.json()
		self.aid = tmp['aid']
		print(tmp)

	def put(self):
		url = "http://robotx.cowbon.info/anchorwaypoints/"+str(self.aid)
		payload = 'mid='+str(self.mid)+'&globalx='+str(self.globalx)+'&globaly='+str(self.globaly)+'&uwbid='+str(self.uwbid)
		headers = {'Content-Type': 'application/x-www-form-urlencoded'}
		response = requests.request("PUT", url, headers=headers, data = payload)
		print(response.text.encode('utf8'))

	def get(self):
		url = "http://robotx.cowbon.info/anchorwaypoints/"+str(self.aid)
		payload = {}
		headers= {}
		response = requests.request("GET", url, headers=headers, data = payload)
		tmp = response.json()
		if not 'aid' in tmp.keys() :
			print('ERROR !!! No aid ??????')
			return
		self.mid = tmp['mid']
		self.globalx = tmp['globalx']
		self.globaly = tmp['globaly']
		self.uwbid = tmp['uwbid']


def rotate(degree, x, y):
  theta = np.radians(degree)
  c, s = np.cos(theta), np.sin(theta)
  R = np.array(((c, -s), (s, c)))
  P = np.array(((x, y)))
  Pr = np.matmul(R,P)
  return Pr[0], Pr[1]

class BH_Cave:
	def __init__(self):
		self.anchor_list = []

		self.fig, self.ax = plt.subplots()
		self.scat = None
		self.text = None
		self.goal_scat = None

		for i in range(1,999):
			if not rospy.has_param("/loc_a"+str(i)+"/aid"):continue
			x,y = rospy.get_param("/loc_a"+str(i)+"/globalx"),rospy.get_param("/loc_a"+str(i)+"/globaly")
			anc = AnchorWaypoint(mid=1,globalx=x,globaly=y,isanchor=True,uwbid=0)
			self.anchor_list.append(anc)


	def update_plot(self, frame):
		if not self.scat == None:self.scat.remove()
		if not self.text == None:self.text.remove()
		for item in self.anchor_list:
			item.get()
			a_x, a_y = rotate(32, float(item.globalx),float(item.globaly))
			self.scat = plt.scatter(a_x,a_y-1, s=90, c = 'r', marker='^', zorder=3)
			self.text = plt.text(a_x-1, a_y, str(item.aid), fontsize=10, c = 'r')


	def plot_init(self):
		while not rospy.has_param("ui_backend/pkg_path") and not rospy.is_shutdown():
			rospy.loginfo("Wait for ui_backend/pkg_path")
			rospy.sleep(0.5)
		map_path = rospy.get_param("ui_backend/pkg_path") + rospy.get_param("/map")
		im = plt.imread(map_path)
		self.ax.imshow(im, aspect=1, extent=(-73, 2, -19.25, 16))
		cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

	def onclick(self,event):
		if event.dblclick:
			# print(event.x, event.y, event.xdata, event.ydata)
			if not self.goal_scat==None :self.goal_scat.remove()
			self.goal_scat = plt.scatter(event.xdata,event.ydata, s=70, c = 'b', marker='o', zorder=3)


if __name__ == '__main__':
	rospy.init_node("navi_backend")
	bh_cave = BH_Cave()
	ani = FuncAnimation(bh_cave.fig, bh_cave.update_plot, init_func=bh_cave.plot_init)
	plt.show(block=True)
