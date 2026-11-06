#!/usr/bin/env python3
"""Drive the unmodified C++ virtualDbus plugin through its real Qt widgets.

No ROS publisher is created here. DBUS and commands are received for evidence;
only Qt mouse/keyboard events and ordinary widget values cause plugin output.
"""
import argparse
import json
import os
from pathlib import Path
import threading
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--script", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get("ROS_MASTER_URI", "").startswith("http://127.0.0.1:"):
        raise RuntimeError("This test driver requires its explicit localhost master")
    if os.environ.get("QT_QPA_PLATFORM") != "offscreen" and not os.environ.get("BASE_LINK_PRIVATE_DISPLAY"):
        raise RuntimeError("Use offscreen Qt or the session's private display")

    import rospy
    from geometry_msgs.msg import Twist
    from rm_msgs.msg import DbusData, ChassisCmd, LegCmd, LeggedChassisMode, LeggedChassisStatus
    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import QApplication, QWidget, QScrollBar, QDial
    from python_qt_binding.QtTest import QTest
    from rqt_gui.main import Main

    args.output.mkdir(parents=True, exist_ok=True)
    plan = json.loads(args.script.read_text())
    rospy.init_node("base_link_virtual_dbus_audit", disable_signals=True)
    events = (args.output / "gui_events.jsonl").open("w")
    received = (args.output / "received_topics.jsonl").open("w")
    lock = threading.Lock()

    def message_value(value):
        if hasattr(value, "__slots__"):
            return {key: message_value(getattr(value, key)) for key in value.__slots__}
        if isinstance(value, (list, tuple)):
            return [message_value(item) for item in value]
        return value

    def receive(topic, message):
        with lock:
            received.write(json.dumps({"topic": topic, "receive_wall_monotonic": time.monotonic(),
                                       "receive_ros_ns": rospy.Time.now().to_nsec(),
                                       "message": message_value(message)}) + "\n")

    subscribers = []
    for topic, kind in [("/rm_ecat_hw/dbus", DbusData), ("/cmd_vel", Twist), ("/cmd_chassis", ChassisCmd),
                        ("/leg_cmd", LegCmd),
                        ("/controllers/chassis_controller/legged_chassis_mode", LeggedChassisMode),
                        ("/controllers/chassis_controller/legged_chassis_status", LeggedChassisStatus)]:
        subscribers.append(rospy.Subscriber(topic, kind, lambda msg, name=topic: receive(name, msg), queue_size=200))

    class DriverMain(Main):
        def create_application(self, argv):
            app = super().create_application(argv)
            self.timer = QTimer(app)
            self.timer.timeout.connect(self.tick)
            self.timer.start(10)
            self.form = None
            self.started = None
            self.index = 0
            self.pressed = set()
            return app

        def find_class(self, parent, name):
            for widget in parent.findChildren(QWidget):
                if widget.metaObject().className() == name:
                    return widget
            raise RuntimeError("Real plugin widget missing: " + name)

        def tick(self):
            try:
                if self.form is None:
                    for candidate in QApplication.allWidgets():
                        if candidate.objectName() == "Form" and candidate.findChild(QScrollBar, "right_switch"):
                            self.form = candidate
                            self.keyboard = self.find_class(candidate, "KeyboardButton")
                            self.slip = self.find_class(candidate, "SlipButton")
                            self.mouse = self.find_class(candidate, "MouseButton")
                            if self.mouse.isChecked():
                                QTest.mouseClick(self.mouse, Qt.LeftButton)
                            candidate.findChild(QDial, "wheel").setValue(50)
                            self.form.grab().save(str(args.output / "virtual_dbus_initial.png"))
                            break
                    return
                now = rospy.Time.now().to_sec()
                if now <= 0.:
                    return
                if self.started is None:
                    self.started = now
                elapsed = now - self.started
                while self.index < len(plan["events"]) and elapsed >= plan["events"][self.index]["at"]:
                    event = plan["events"][self.index]
                    self.apply(event)
                    events.write(json.dumps({"sim_time": now, "elapsed": elapsed, "event": event}) + "\n")
                    events.flush()
                    self.index += 1
                if elapsed >= plan["duration"]:
                    for key in sorted(self.pressed):
                        QTest.keyRelease(self.keyboard, getattr(Qt, "Key_" + key))
                    self.pressed.clear()
                    self.form.grab().save(str(args.output / "virtual_dbus_final.png"))
                    self.timer.stop()
                    QApplication.quit()
            except Exception as error:
                events.write(json.dumps({"error": str(error)}) + "\n")
                events.flush()
                self.timer.stop()
                QApplication.exit(2)

        def apply(self, event):
            operation = event["op"]
            if operation == "switch":
                self.form.findChild(QScrollBar, event["side"] + "_switch").setValue(event["value"])
            elif operation == "publish_toggle":
                QTest.mouseClick(self.slip, Qt.LeftButton)
            elif operation == "keyboard_on":
                if not self.keyboard.isChecked():
                    QTest.mouseClick(self.keyboard, Qt.LeftButton)
            elif operation in ("press", "release"):
                key = event["key"]
                if operation == "press":
                    QTest.keyPress(self.keyboard, getattr(Qt, "Key_" + key))
                    self.pressed.add(key)
                else:
                    QTest.keyRelease(self.keyboard, getattr(Qt, "Key_" + key))
                    self.pressed.discard(key)
            else:
                raise ValueError("Unknown GUI operation " + operation)

    try:
        gui = DriverMain(filename="rqt_virtual_dbus", settings_filename="base_link_control_test")
        return gui.main(argv=["rqt_virtual_dbus", "--force-discover"], standalone="rqt_virtual_dbus")
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        events.close()
        received.close()


if __name__ == "__main__":
    raise SystemExit(main())
