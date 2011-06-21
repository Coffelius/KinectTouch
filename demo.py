from KinectTouch import KinectTouch
from kivy.uix.button import Button
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.image import Image
import pygame
from kivy.config import Config
from kivy.animation import Animation
import time

class MyButton(Button):
    def __init__(self):
        Button.__init__(self);
        self.start=None
        self.frame=0

    def on_touch_down(self, touch):
        self.start=time.time()
        Button.on_touch_down(self, touch)


    def on_touch_up(self, touch):
        Button.on_touch_up(self, touch)
        self.start=None;
    def on_touch_move(self, touch):
        if not self.collide_point(touch.x, touch.y):
            self.start=None
            return False

        Button.on_touch_move(self, touch)
        if(self.start==None):
            self.start=time.time()
        elif(time.time()-self.start>=2):
            Button.on_touch_up(self, touch)
        elif(time.time()-self.start>=1):
            Button.on_touch_down(self, touch)

        return True

    def draw(self):
        timepushing=0;
        if(self.start!=None):
            timepushing=time.time()-self.start;

        self.frame+=1
        Button.draw(self)

class MainWidget(Widget):
	def __init__(self):
		Widget.__init__(self, draw_children=False);
		self.cursorpos=None
		self.cursor=None
		self.register_event_type('on_air')

	def on_touch_down(self, touch):
		self.cursorpos=touch.pos
		x, y = self.cursorpos
		self.cursor.pos=(x-40, y-40);
		self.cursor.visible=True;
		Widget.on_touch_down(self, touch)
		self.on_air(touch)

	def on_air(self, touch):
		for child in self.children[:]:
			if child.dispatch('on_air', touch):
				return True


	def on_touch_up(self, touch):
		self.cursor.visible=False;
		Widget.on_touch_up(self, touch)


	def on_touch_move(self, touch):
		self.cursorpos=touch.pos;
		x, y = self.cursorpos
		self.cursor.pos=(x-40, y-40);
		Widget.on_touch_move(self, touch)
		self.on_air(touch)

class Page(Image):
	pass

class Catalog(Widget):
	def __init__(self, **kwargs):
		Widget.__init__(self, **kwargs)
		self.pages=[]
		self.currentpage=0
		self.lastpage=-1
		self.lastupdated=None


	def addPage(self, filename):
		img=Page(source=filename, pos=self.posByIndex(len(self.pages)+1),size=(self.size[0]*1, self.size[1]*1))
		self.pages.append(img);

	def blocked(self):
		if(self.lastupdated==None):
			return False
		now=time.time()
		if(now-self.lastupdated>=2):
			return False
		return True

	def nextPage(self):
		if(self.blocked()):
			return;
		self.lastpage=self.currentpage
		self.currentpage+=1
		if(self.currentpage>=len(self.pages)):
			self.currentpage=len(self.pages)-1
		self.update()

	def lastPage(self):
		if(self.blocked()):
			return;
		self.lastpage=self.currentpage
		self.currentpage-=1
		if(self.currentpage<0):
			self.currentpage=0
		self.update()

	def posByIndex(self, i):
		return (i*self.size[0], 0);

	def updated(self):
		self.lastupdated=time.time()
	def on_air(self, touch):
		if(touch.pos[0]>self.size[0]*0.8):
			self.nextPage()
		if(touch.pos[0]<self.size[0]*0.2):
			self.lastPage()

	def update(self):
		if(self.currentpage==self.lastpage):
			return
		self.updated()

		self.clear_widgets()

		page=self.pages[self.currentpage]
		if(self.lastpage!=-1):
			lastpage=self.pages[self.lastpage];
			lastpage.pos=self.posByIndex(0)
			#lastpage.opacity=1.0
			self.add_widget(lastpage)

		else:
			lastpage=None

		#page.opacity=0.0
		if(self.currentpage>self.lastpage):
			page.pos=self.posByIndex(1)
			if(lastpage!=None):
				anim2=Animation(pos=self.posByIndex(-1),  d=0.5, t="in_quad")
				anim2.start(lastpage)
		else:
			page.pos=self.posByIndex(-1)
			if(lastpage!=None):
				anim2=Animation(pos=self.posByIndex(1),  d=0.5, t="in_quad")
				anim2.start(lastpage)




		self.add_widget(page)


		anim=Animation(pos=self.posByIndex(0), d=0.5, t="in_quad")
		anim.start(page)



class RabatApp(App):
	def build(self):
		size=(Config.getint("graphics", "width"), Config.getint("graphics", "height"))
		main=MainWidget()
		cursor=Image(source="cursor.png");
		main.cursor=cursor;
		cursor.visible=False;

		#background=Page(source="images/background.png", size=size)

		print size
		cat=Catalog(pos=(0,0), size=size);
		cat.addPage("marker.png");
		cat.update()

		#main.add_widget(background);
		main.add_widget(cat);
		main.add_widget(cursor);

		return main

RabatApp().run()
