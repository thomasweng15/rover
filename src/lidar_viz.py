#Display Data from Neato LIDAR
#based on code from Nicolas "Xevel" Saugnier
#requires vpython

from rover.msg import LidarPoint
from visual import *
import rospy
import math

OFFSET = 8000

class LidarViz:
	def __init__(self, offset):

		self.use_outer_line = False 
		self.use_lines = False
		self.use_points = True
		self.use_intensity = True
		self.offset = offset

		self.init_viz()

	def init_viz(self):
		# sample and intensity points
		self.point1 = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0 , 1, 0))
		self.point1b = points(pos=[(0,0,0) for i in range(360)], size=5, color=(0.4, 0, 0))
		self.point2 = points(pos=[(0,0,0) for i in range(360)], size=3, color=(1 , 1, 0))
		self.point2b = points(pos=[(0,0,0) for i in range(360)], size=3, color=(0.4, 0.4, 0))
		
		# lines
		self.outer_line = curve(pos=[(0,0,0) for i in range(360)], size=5, color=(1 , 0, 0))
		self.lines = [curve(pos=[(self.offset*cos(i* pi / 180.0),0,self.offset*-sin(i* pi / 180.0)),(self.offset*cos(i* pi / 180.0),0,self.offset*-sin(i* pi / 180.0))], color=[(0.1, 0.1, 0.2),(1,0,0)]) for i in range(360)]
		self.zero_intensity_ring = ring(pos=(0,0,0), axis=(0,1,0), radius=self.offset-1, thickness=1, color = color.yellow)

		self.label_speed = label(pos = (0,-500,0), xoffset=1, box=False, visible = False)
		self.label_errors = label(pos = (0,-1000,0), xoffset=1, text="errors: 0", visible = False, box=False)

	def update_speed(self, speed_rpm):
		self.label_speed.text = "RPM: " + str(speed_rpm)

	def redraw(self, x1, angle, dist_mm, quality):
		angle_rad = angle * math.pi / 180.0
		c = math.cos(angle_rad)
		s = -math.sin(angle_rad)

		dist_x = dist_mm * c
		dist_y = dist_mm * s

		# reset the point display
		self.point1.pos[angle] = vector(0, 0, 0)
		self.point1b.pos[angle] = vector(0, 0, 0)
		self.point2.pos[angle] = vector(0, 0, 0)
		self.point2b.pos[angle] = vector(0, 0, 0)

		if not self.use_lines: 
			self.lines[angle].pos[1] = (self.offset * c, 0, self.offset * s)
		
		if not self.use_outer_line:
			self.outer_line.pos[angle] = (self.offset * c, 0, self.offset * s)
			self.outer_line.color[angle] = (0.1, 0.1, 0.2)

		# display the sample
		if x1 & 0x80: # bad data
			self.lines[angle].pos[1] = (self.offset * c, 0, self.offset * s)
			self.outer_line.pos[angle] = (self.offset * c, 0, self.offset * s)
			self.outer_line.color[angle] = (0.1, 0.1, 0.2)
		else:
			if not x1 & 0x40: # quality is good
				if self.use_points: self.point1.pos[angle] = vector(dist_x, 0, dist_y)
				if self.use_intensity: self.point2.pos[angle] = vector((quality + self.offset) * c, 0, (quality + self.offset) * s)
				
				if self.use_lines: self.lines[angle].color[1] = (1, 0, 0)
				if self.use_outer_line: self.outer_line.color[angle] = (1, 0, 0)
			else: # quality not as good as expected
				if self.use_points: self.point1b.pos[angle] = vector(dist_x, 0, dist_y)
				if self.use_intensity: self.point2b.pos[angle] = vector((quality + self.offset) * c, 0, (quality + self.offset) * s)

				if self.use_lines: self.lines[angle].color[1] = (0.4, 0, 0)
				if self.use_outer_line: self.outer_line.color[angle] = (0.4, 0, 0)

			if self.use_lines: self.lines[angle].pos[1] = (dist_x, 0, dist_y)
			if self.use_outer_line:	self.outer_line.pos[angle] = (dist_x, 0, dist_y)

	def checkKeys(self):
		if scene.kb.keys: 
			s = scene.kb.getkey()

			if s=="o": # Toggle outer line
				print "Toggle outer line"
				self.use_outer_line = not self.use_outer_line	
			elif s == "l": # Toggle rays
				print "Toggle rays"
				self.use_lines = not self.use_lines
			elif s == "p": # Toggle points
				self.use_points = not self.use_points
			elif s == "i": # Toggle intensity
				self.use_intensity = not self.use_intensity
				self.zero_intensity_ring.visible = self.use_intensity
			elif s == "j": # Toggle rpm
				print "Toggle rpm"
				self.label_speed.visible = not self.label_speed.visible
			elif s == "k": # Toggle errors
				self.label_errors.visible = not self.label_errors.visible

