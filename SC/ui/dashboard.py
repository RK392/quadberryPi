import threading
import time
import logging

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle
from kivy.clock import mainthread
from SC.util.constants import *

STATE_IMU_READING = 'reading'


class SidePanel(GridLayout):

    def __init__(self, **kwargs):
        super(SidePanel, self).__init__(**kwargs)

        with self.canvas.before:
            Color(0.2, 0.2, 0.2, 1)  # green; colors range from 0-1 instead of 0-255
            self.rect = Rectangle(size=self.size, pos=self.pos)

        def update_rect(instance, value):
            instance.rect.pos = instance.pos
            instance.rect.size = instance.size

        # listen to size and position changes
        self.bind(pos=update_rect, size=update_rect)


class MainScreen(BoxLayout):

    def __init__(self, **kwargs):
        super(MainScreen, self).__init__(**kwargs)
        self.orientation = 'horizontal'

        self.left_panel = SidePanel()
        self.left_panel.cols = 2
        self.left_panel.size_hint = (0.75, 1)

        self.right_panel = SidePanel()
        self.right_panel.cols = 2
        self.right_panel.size_hint = (0.75, 1)

        self.center_panel = BoxLayout()
        self.center_panel.size_hint = (1, 1)

        self.add_widget(self.left_panel)
        self.add_widget(self.center_panel)
        self.add_widget(self.right_panel)

        # self.user_label = Label(text="User Name")
        #

        self.fields = {}

        self.title_label = Label(text="Das Auto", bold=True, font_size=36)
        self.center_panel.add_widget(self.title_label)

        self.left_panel.add_widget(Label(text='Remote Throttle'))
        self.fields[STATE_THROTTLE_REMOTE] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_THROTTLE_REMOTE])
        self.left_panel.add_widget(Label(text='Current Throttle'))
        self.fields[STATE_THROTTLE_CURRENT] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_THROTTLE_CURRENT])
        self.left_panel.add_widget(Label(text='Remote Brake'))
        self.fields[STATE_BRAKE_REMOTE] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_BRAKE_REMOTE])
        self.left_panel.add_widget(Label(text='Current Brake'))
        self.fields[STATE_BRAKE_CURRENT] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_BRAKE_CURRENT])
        self.left_panel.add_widget(Label(text='Target Steering'))
        self.fields[STATE_STEERING_TARGET] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_STEERING_TARGET])
        self.left_panel.add_widget(Label(text='Current Steering'))
        self.fields[STATE_STEERING_CURRENT] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_STEERING_CURRENT])
        self.left_panel.add_widget(Label(text="Front Left RPM"))
        self.fields[STATE_FRONT_LEFT_RPM] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_FRONT_LEFT_RPM])
        self.left_panel.add_widget(Label(text='Front Right RPM'))
        self.fields[STATE_FRONT_RIGHT_RPM] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_FRONT_RIGHT_RPM])
        self.left_panel.add_widget(Label(text='Rear Left RPM'))
        self.fields[STATE_REAR_LEFT_RPM] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_REAR_LEFT_RPM])
        self.left_panel.add_widget(Label(text='Rear Right RPM'))
        self.fields[STATE_REAR_RIGHT_RPM] = Label(text='0')
        self.left_panel.add_widget(self.fields[STATE_REAR_RIGHT_RPM])

        self.right_panel.add_widget(Label(text='Gear'))
        self.fields[STATE_GEAR] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_GEAR])
        self.right_panel.add_widget(Label(text='Driving Mode'))
        self.fields[STATE_DRIVING_MODE] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_DRIVING_MODE])
        self.right_panel.add_widget(Label(text='Horn'))
        self.fields[STATE_HORN] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_HORN])
        self.right_panel.add_widget(Label(text='Headlights'))
        self.fields[STATE_HEAD_LIGHTS] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_HEAD_LIGHTS])
        self.right_panel.add_widget(Label(text='Other Lights'))
        self.fields[STATE_OTHER_LIGHTS] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_OTHER_LIGHTS])
        self.right_panel.add_widget(Label(text='Heading'))
        self.fields[STATE_IMU_READING_HEADING] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_IMU_READING_HEADING])
        self.right_panel.add_widget(Label(text='Roll'))
        self.fields[STATE_IMU_READING_ROLL] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_IMU_READING_ROLL])
        self.right_panel.add_widget(Label(text='Pitch'))
        self.fields[STATE_IMU_READING_PITCH] = Label(text='0')
        self.right_panel.add_widget(self.fields[STATE_IMU_READING_PITCH])
        # self.fields[STATE_IMU_DATA] = None

    @mainthread
    def update_data(self, state_map):
        logging.debug('Update UI Data...')
        for key in state_map:
            value = state_map[key]
            if key in self.fields:
                if key == STATE_GEAR:
                    self.fields[key].text = (
                        'D' if value == GEAR_DRIVE
                        else 'P' if value == GEAR_PARK
                        else 'R' if value == GEAR_REVERSE
                        else 'N')
                elif key in (STATE_HEAD_LIGHTS, STATE_HORN, STATE_OTHER_LIGHTS):
                    self.fields[key].text = 'ON' if value == 1 else 'OFF'
                elif key in (STATE_INDICATOR_MODE, STATE_DRIVING_MODE):
                    self.fields[key].text = value.upper()
                # elif key == STATE_IMU_DATA:
                #     heading, roll, pitch = value[STATE_IMU_READING]
                #     self.fields['heading'].text = str(heading)
                #     self.fields['roll'].text = str(roll)
                #     self.fields['pitch'].text = str(pitch)
                else:
                    self.fields[key].text = str(value)


class DashboardApp(App):

    def __init__(self):
        super(DashboardApp, self).__init__()
        self.main_screen = None

    def build(self):
        self.main_screen = MainScreen()
        return self.main_screen


if __name__ == '__main__':
    app = DashboardApp()
    thread = threading.Thread(target=app.run, args=())
    thread.start()
    time.sleep(2)
    app.main_screen.fields['gear'].text = "D"
    time.sleep(2)
    app.main_screen.fields['gear'].text = "N"
    time.sleep(1)
    app.stop()
    time.sleep(5)

# app = MyApp()
# thread = threading.Thread(target=app.run, args=())
# thread.start()
# app.stop()
#    def update_data(self, state_map):

