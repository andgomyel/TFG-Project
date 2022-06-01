# -*- coding: utf-8 -*-

from numpy import inf
from sympy import N
import sys
import threading

from PyQt5 import QtGui
from PyQt5.QtWidgets import QTextEdit, QWidget, QApplication, QGridLayout, QLabel, QPushButton, QHBoxLayout, QVBoxLayout
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap

from camera import camera
from state_machine import *

# user_input = get_key_input()

class Interface(QWidget):

    window = QApplication(sys.argv)
    
    def __init__(self):
        super().__init__()

        self.total_objects = 0

        self.n_robots = 3
        self.cameras = []
        for id in range(0,self.n_robots):
            self.cameras.append(camera(id))
        
        self.sm = State_Machine()
        
        self.setupUi()

    def setupUi(self):

        self.setWindowTitle("Interfaz de usuario")

        layout = QGridLayout()

        #################
        
        top_cameras = QHBoxLayout()
        bottom_cameras = QHBoxLayout()

        legend = QHBoxLayout()

        ################
        legend_layout = QVBoxLayout()
        self.cont_objects = QTextEdit("Suspicious objects found: 0")
        self.cont_objects.setFixedWidth(300)
        self.cont_objects.setFixedHeight(30)
        self.cont_objects.setFrameStyle(0)
        self.cont_objects.setReadOnly(True)
        self.cont_objects.setStyleSheet("background-color: #3D3D3D; font-weight: bold; color: #FFFFFF")
        self.cont_objects.setAlignment(Qt.AlignCenter)
        legend_layout.addWidget(self.cont_objects)
        
        # Bloque para los widgets de las cámaras de los robots
        self.cam_titles = []
        self.camera_widgets = []
        for i in range(self.n_robots):
            self.cam_titles.append(QLabel("Robot "+str(i)+" Camera"))
            self.cam_titles[i].setStyleSheet("font-weight: bold; color: #FFFFFF")
            self.cam_titles[i].setAlignment(Qt.AlignCenter)
            
            self.camera_widgets.append(QLabel("self"))
            self.camera_widgets[i].setFixedSize(640, 360)
            self.camera_widgets[i].setScaledContents(True)

        ####################
        camera_layout = []
        for i in range(self.n_robots):
            camera_layout.append(QVBoxLayout())
            camera_layout[i].addWidget(self.cam_titles[i])
            camera_layout[i].addWidget(self.camera_widgets[i])

       
        top_cameras.addLayout(camera_layout[0])
        top_cameras.addLayout(camera_layout[1])
        bottom_cameras.addLayout(camera_layout[2])

        legend.addLayout(legend_layout)

        
        ####################
        layout.addLayout(top_cameras, 0, 0)
        layout.addLayout(bottom_cameras, 1, 0)
        layout.addLayout(legend, 2, 0)

        self.setLayout(layout)
        self.setBaseSize(1350,780)

        p = self.palette()
        p.setColor(self.backgroundRole(), QtGui.QColor(100,100,100))   # 51,51,51
        self.setPalette(p)

        self.show()

        self.create_connections()

    def updateCamera(self,id):
         
        if self.cameras[id].detecting:
            img = self.cameras[id].yolo_img

        else:
            img = self.cameras[id].original_img

        height, width, channel = img.shape
        step = channel*width
        qImg = QImage(img.data, width, height, step, QImage.Format_RGB888).rgbSwapped()

        self.camera_widgets[id].setPixmap(QPixmap.fromImage(qImg))

    def computeObjects(self):
        n = 0
        for cam in self.cameras:
            n += cam.marker_id
            print(str(cam.ID) + str(cam.marker_id))
        return n

    def updateCounter(self):

        self.total_objects = self.computeObjects()
        self.cont_objects.setText("Suspicious objects found: " + str(self.total_objects))
        
    def update_real_time(self):

        self.thread.start()
        self.window.exec_()

    def create_connections(self):

        self.cameras[0].image_signal.connect(lambda: self.updateCamera(0))
        self.cameras[1].image_signal.connect(lambda: self.updateCamera(1))
        self.cameras[2].image_signal.connect(lambda: self.updateCamera(2))
        self.cameras[0].update_signal.connect(lambda: self.updateCounter())
        self.cameras[1].update_signal.connect(lambda: self.updateCounter())
        self.cameras[2].update_signal.connect(lambda: self.updateCounter())
        self.thread = threading.Thread(target=self.sm.run, args=())

    def closeEvent(self, _):
        self.thread.join()

    # def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
    #     return super().closeEvent(a0)
        
if __name__ == "__main__":
    
    ui = Interface()
    
    ui.update_real_time()
