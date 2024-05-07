from PySide6.QtWidgets import (
    QApplication,
    QGraphicsScene,
    QMainWindow,
)
from PySide6.QtGui import (
    QColor,
    QPolygonF,
    QMouseEvent,
)
from PySide6.QtCore import QPoint

import sys
import math
import rclpy
from threading import Thread
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from elkapod_msgs.msg import TrajectoryParameters
from .elkapod_controller_ui import Ui_HexapodController


class ApplicationMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_HexapodController()
        self.ui.setupUi(self)

        self.setup()

    def setup(self):
        self.leg_spacing_scene = self.draw_leg_spacing_scene(40)
        self.ui.leg_spacing_view.setScene(self.leg_spacing_scene)
        self.height_scene = self.draw_height_scene(20)
        self.ui.height_view.setScene(self.height_scene)
        self.corpus_position_scene = self.draw_corpus_position_scene((0., 0.))
        self.ui.corpus_position.setScene(self.corpus_position_scene)
        self.ui.corpus_position.mousePressEvent = \
            self.update_corpus_position

        self.trajectory_parameters = TrajectoryParameters()
        self.trajectory_parameters.leg_spacing = 0.6
        self.trajectory_parameters.height = 0.1
        self.trajectory_parameters.vdir = math.pi/2
        self.trajectory_parameters.vval = 0.
        self.trajectory_parameters.omega = 0.
        self.trajectory_parameters.yaw = 0.
        self.trajectory_parameters.pitch = 0.
        self.trajectory_parameters.roll = 0.
        self.trajectory_parameters.step_height = 0.
        self.trajectory_parameters.corpus_position = [0., 0.]
        self.trajectory_parameters.cycle_time = 6.
        self.trajectory_parameters.supportive_legs = [True for _ in range(6)]
        self.trajectory_parameters.gait = "3POINT"

        self.ui.leg_spacing_slider.sliderMoved.connect(self.update_leg_spacing)
        self.ui.height_slider.sliderMoved.connect(self.update_height)
        self.ui.vdir_dial.sliderMoved.connect(self.update_vdir)
        self.ui.vval_slider.sliderMoved.connect(self.update_vval)
        self.ui.omega_dial.sliderMoved.connect(self.update_omega)
        self.ui.yaw_dial.sliderMoved.connect(self.update_yaw)
        self.ui.pitch_dial.sliderMoved.connect(self.update_pitch)
        self.ui.roll_dial.sliderMoved.connect(self.update_roll)
        self.ui.step_height_slider.sliderMoved.connect(self.update_step_height)

        self.ui.gait_selection.addItems([
            '3POINT',
            'RIPPLE',
            'MECHATRONIC',
            'STAND',
        ])
        self.ui.gait_selection.currentTextChanged.connect(self.update_gait)

        self.sl_checkboxes = [self.ui.checkBox_1, self.ui.checkBox_2,
                              self.ui.checkBox_3, self.ui.checkBox_4,
                              self.ui.checkBox_5, self.ui.checkBox_6]
        for checkbox in self.sl_checkboxes:
            checkbox.setEnabled(False)
            checkbox.setChecked(True)
            checkbox.toggled.connect(self.update_supportive_legs)

        self.allowed_non_sl = 0

    def update_leg_spacing(self, spacing):
        self.leg_spacing_scene = self.draw_leg_spacing_scene((spacing-40)//2)
        self.ui.leg_spacing_view.setScene(self.leg_spacing_scene)

        self.trajectory_parameters.leg_spacing = float(spacing)*0.005
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_height(self, height):
        self.height_scene = self.draw_height_scene(height//5)
        self.ui.height_view.setScene(self.height_scene)

        self.trajectory_parameters.height = float(height)*0.001
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_corpus_position(self, event: QMouseEvent):
        position = (
            self.ui.corpus_position.mapToScene(event.position().toPoint()).x(),
            self.ui.corpus_position.mapToScene(event.position().toPoint()).y(),
        )
        self.corpus_position_scene = self.draw_corpus_position_scene(position)
        self.ui.corpus_position.setScene(self.corpus_position_scene)
        self.trajectory_parameters.corpus_position = [
            float(position[0])*0.000833,
            float(position[1])*0.000833,
        ]
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_vdir(self, vdir):
        self.trajectory_parameters.vdir = ((float(vdir)+90.)/360.)*2*math.pi
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_vval(self, vval):
        self.trajectory_parameters.vval = float(vval)*0.0003
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_omega(self, omega):
        self.trajectory_parameters.omega = -(float(omega)*2*math.pi/3600)
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_yaw(self, yaw):
        self.trajectory_parameters.yaw = float(yaw)*2*math.pi/3600
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_pitch(self, pitch):
        self.trajectory_parameters.pitch = float(pitch)*2*math.pi/3600
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_roll(self, roll):
        self.trajectory_parameters.roll = float(roll)*2*math.pi/3600
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_step_height(self, step_height):
        self.trajectory_parameters.step_height = float(step_height)/10000.
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_gait(self, gait):
        if gait == 'MECHATRONIC':
            self.allowed_non_sl = 1
            if sum(self.trajectory_parameters.supportive_legs) < 5:
                for checkbox in self.sl_checkboxes:
                    checkbox.setEnabled(False)

                for checkbox in self.sl_checkboxes:
                    if not checkbox.isChecked():
                        checkbox.setChecked(True)
                        break
            elif sum(self.trajectory_parameters.supportive_legs) == 5:
                for checkbox in self.sl_checkboxes:
                    if checkbox.isChecked():
                        checkbox.setEnabled(False)
                    else:
                        checkbox.setEnabled(True)
            elif sum(self.trajectory_parameters.supportive_legs) >= 6:
                for checkbox in self.sl_checkboxes:
                    checkbox.setEnabled(True)
        elif gait == 'STAND':
            self.allowed_non_sl = 2

            if sum(self.trajectory_parameters.supportive_legs) == 4:
                for checkbox in self.sl_checkboxes:
                    if checkbox.isChecked():
                        checkbox.setEnabled(False)
                    else:
                        checkbox.setEnabled(True)
            elif sum(self.trajectory_parameters.supportive_legs) >= 5:
                for checkbox in self.sl_checkboxes:
                    checkbox.setEnabled(True)
        else:
            self.allowed_non_sl = 0
            for checkbox in self.sl_checkboxes:
                checkbox.setEnabled(False)
                if not checkbox.isChecked():
                    checkbox.setChecked(True)

        self.trajectory_parameters.gait = gait
        self.send_trajectory_parameters(self.trajectory_parameters)

    def update_supportive_legs(self):
        self.trajectory_parameters.supportive_legs[int(self.sender().objectName()[-1])-1] = \
            self.sender().isChecked()

        if len(self.trajectory_parameters.supportive_legs) - sum(self.trajectory_parameters.supportive_legs) >= \
           self.allowed_non_sl:
            for checkbox in self.sl_checkboxes:
                if checkbox.isChecked():
                    checkbox.setEnabled(False)
                else:
                    checkbox.setEnabled(True)
        else:
            for checkbox in self.sl_checkboxes:
                checkbox.setEnabled(True)

        self.send_trajectory_parameters(self.trajectory_parameters)

    def draw_corpus_position_scene(self, position):
        scene = QGraphicsScene(self)
        scene.addLine(-50., 0., 50., 0.)
        scene.addLine(0., -80., 0., 80.)
        scene.addEllipse(
            position[0]-2., position[1]-2., 4., 4.,
            QColor(255, 0, 0, 255),
            QColor(255, 0, 0, 255),
        )

        return scene

    def draw_leg_spacing_scene(self, leg_len):
        scene = QGraphicsScene(self)

        corpus = QPolygonF([
            QPoint(0, 0),
            QPoint(40, 0),
            QPoint(40, 60),
            QPoint(0, 60),
        ])
        leg1 = QPolygonF([
            QPoint(-leg_len, 0),
            QPoint(0, 0),
            QPoint(0, 5),
            QPoint(-leg_len, 5),
        ])
        leg2 = QPolygonF([
            QPoint(-leg_len, 28),
            QPoint(0, 28),
            QPoint(0, 33),
            QPoint(-leg_len, 33),
        ])
        leg3 = QPolygonF([
            QPoint(-leg_len, 55),
            QPoint(0, 55),
            QPoint(0, 60),
            QPoint(-leg_len, 60),
        ])
        leg4 = QPolygonF([
            QPoint(40, 0),
            QPoint(40+leg_len, 0),
            QPoint(40+leg_len, 5),
            QPoint(40, 5),
        ])
        leg5 = QPolygonF([
            QPoint(40, 28),
            QPoint(40+leg_len, 28),
            QPoint(40+leg_len, 33),
            QPoint(40, 33),
        ])
        leg6 = QPolygonF([
            QPoint(40, 55),
            QPoint(40+leg_len, 55),
            QPoint(40+leg_len, 60),
            QPoint(40, 60),
        ])

        scene.addPolygon(corpus,
                         QColor(255, 0, 0, 255),
                         QColor(255, 0, 0, 255))
        scene.addPolygon(leg1,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg2,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg3,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg4,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg5,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leg6,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))

        return scene

    def draw_height_scene(self, height):
        scene = QGraphicsScene(self)

        corpus = QPolygonF([
            QPoint(0, -3),
            QPoint(40, -3),
            QPoint(40, 7),
            QPoint(0, 7),
        ])
        leglt = QPolygonF([
            QPoint(-40, 0),
            QPoint(0, 0),
            QPoint(0, 5),
            QPoint(-40, 5),
        ])
        legrt = QPolygonF([
            QPoint(40, 0),
            QPoint(80, 0),
            QPoint(80, 5),
            QPoint(40, 5),
        ])
        leglb = QPolygonF([
            QPoint(-40, 5),
            QPoint(-35, 5),
            QPoint(-35, 5+height),
            QPoint(-40, 5+height),
        ])
        legrb = QPolygonF([
            QPoint(75, 5),
            QPoint(80, 5),
            QPoint(80, 5+height),
            QPoint(75, 5+height),
        ])

        scene.addPolygon(corpus,
                         QColor(255, 0, 0, 255),
                         QColor(255, 0, 0, 255))
        scene.addPolygon(leglt,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(leglb,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(legrt,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))
        scene.addPolygon(legrb,
                         QColor(0, 0, 0, 255),
                         QColor(0, 0, 0, 255))

        return scene


class ElkapodControllerGui(Node):
    def __init__(self):
        super().__init__("ElkapodControllerGui")

        self._publisher = self.create_publisher(
            TrajectoryParameters,
            "elkapod_trajectory_parameters",
            10,
        )

    def send_trajectory_parameters(self, params):
        self._publisher.publish(params)


def main(args=None):
    rclpy.init(args=args)

    app = QApplication()
    window = ApplicationMainWindow()
    ros_node = ElkapodControllerGui()

    window.send_trajectory_parameters = (
        lambda params: ros_node.send_trajectory_parameters(params)
    )
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        window.show()
        sys.exit(app.exec())
    finally:
        ros_node.destroy_node()
        executor.shutdown()


if __name__ == "__main__":
    main()
