import threading
import time
import logging

from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.slider import Slider
from kivy.uix.progressbar import ProgressBar
from kivy.graphics import Color, Rectangle
from kivy.clock import Clock, mainthread


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
        self.fields['throttle_remote'] = Label(text='0')
        self.left_panel.add_widget(self.fields['throttle_remote'])
        self.left_panel.add_widget(Label(text='Current Throttle'))
        self.fields['throttle_current'] = Label(text='0')
        self.left_panel.add_widget(self.fields['throttle_current'])
        self.left_panel.add_widget(Label(text='Remote Brake'))
        self.fields['brake_remote'] = Label(text='0')
        self.left_panel.add_widget(self.fields['brake_remote'])
        self.left_panel.add_widget(Label(text='Current Brake'))
        self.fields['brake_current'] = Label(text='0')
        self.left_panel.add_widget(self.fields['brake_current'])
        self.left_panel.add_widget(Label(text='Target Steering'))
        self.fields['steering_target'] = Label(text='0')
        self.left_panel.add_widget(self.fields['steering_target'])
        self.left_panel.add_widget(Label(text='Current Steering'))
        self.fields['steering_current'] = Label(text='0')
        self.left_panel.add_widget(self.fields['steering_current'])
        self.left_panel.add_widget(Label(text='Front Left RPM'))
        self.fields['fl_rpm'] = Label(text='0')
        self.left_panel.add_widget(self.fields['fl_rpm'])
        self.left_panel.add_widget(Label(text='Front Right RPM'))
        self.fields['fr_rpm'] = Label(text='0')
        self.left_panel.add_widget(self.fields['fr_rpm'])
        self.left_panel.add_widget(Label(text='Rear Left RPM'))
        self.fields['rl_rpm'] = Label(text='0')
        self.left_panel.add_widget(self.fields['rl_rpm'])
        self.left_panel.add_widget(Label(text='Rear Right RPM'))
        self.fields['rr_rpm'] = Label(text='0')
        self.left_panel.add_widget(self.fields['rr_rpm'])

        self.right_panel.add_widget(Label(text='Gear'))
        self.fields['gear'] = Label(text='0')
        self.right_panel.add_widget(self.fields['gear'])
        self.right_panel.add_widget(Label(text='Driving Mode'))
        self.fields['driving_mode'] = Label(text='0')
        self.right_panel.add_widget(self.fields['driving_mode'])
        self.right_panel.add_widget(Label(text='Horn'))
        self.fields['horn'] = Label(text='0')
        self.right_panel.add_widget(self.fields['horn'])
        self.right_panel.add_widget(Label(text='Headlights'))
        self.fields['head_lights'] = Label(text='0')
        self.right_panel.add_widget(self.fields['head_lights'])
        self.right_panel.add_widget(Label(text='Other Lights'))
        self.fields['other_lights'] = Label(text='0')
        self.right_panel.add_widget(self.fields['other_lights'])
        self.right_panel.add_widget(Label(text='Heading'))
        self.fields['heading'] = Label(text='0')
        self.right_panel.add_widget(self.fields['heading'])
        self.right_panel.add_widget(Label(text='Roll'))
        self.fields['roll'] = Label(text='0')
        self.right_panel.add_widget(self.fields['roll'])
        self.right_panel.add_widget(Label(text='Pitch'))
        self.fields['pitch'] = Label(text='0')
        self.right_panel.add_widget(self.fields['pitch'])
        self.fields['imu_data'] = None
        # self.user_label.size_hint = (, 1)
        # self.add_widget()
        # self.add_widget(self.username)
        # self.right_panel.add_widget(Label(text='password'))
        # self.password = TextInput(password=True, multiline=False)
        # self.add_widget(self.password)

    @mainthread
    def update_data(self, state_map):
        logging.debug('Update UI Data...')
        for key in state_map:
            value = state_map[key]
            if key in self.fields:
                if key == 'gear':
                    self.fields[key].text = (
                        'D' if value == 'drive'
                        else 'P' if value == 'park'
                        else 'R' if value == 'reverse'
                        else 'N')
                elif key in ('head_lights', 'horn', 'other_lights'):
                    self.fields[key].text = 'ON' if value == 1 else 'OFF'
                elif key in ('indicator_mode', 'driving_mode'):
                    self.fields[key].text = value.upper()
                elif key == 'imu_data':
                    heading, roll, pitch = value['reading']
                    self.fields['heading'].text = str(heading)
                    self.fields['roll'].text = str(roll)
                    self.fields['pitch'].text = str(pitch)
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

