import sys
import logging
import socket
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget
from PyQt5.uic import loadUi
from main_ui import Ui_MainWindow


ROBOT_PORT_1 = 30004    # RTDE
ROBOT_PORT_2 = 29999    # Socket
config_filename = 'config/config-test.xml'

class URCommunication_UI(QMainWindow):
    def __init__(self):
        super(URCommunication_UI, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.ui.serverInput.setText('192.168.189.129')
        self.ui.connectBtn.clicked.connect(self.setup_connection)
        self.ui.disconnectBtn.clicked.connect(self.end_connection)
        
        # Real time connection status tracking every second
        self.rt_con_status = False
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_connection)
        self.timer.start(1000)
        
        self.ui.powerOnBtn.clicked.connect(self.power_on)
        self.ui.powerOffBtn.clicked.connect(self.power_off)
        self.ui.brakeReleaseBtn.clicked.connect(self.brake_release)
        
        # self.ui.startFreeDriveBtn.clicked.connect(self.enable_free_drive)
        # self.ui.endFreeDriveBtn.clicked.connect(self.disable_free_drive)
        self.ui.speedControl.valueChanged.connect(self.update_speed)

    def setup_connection(self):
        try:
            ROBOT_HOST = str(self.ui.serverInput.text())
            
            # ------------ Initialize rtde connection ------------
            self.keep_running = True
            logging.getLogger().setLevel(logging.INFO)
            
            conf = rtde_config.ConfigFile(config_filename)
            state_names, state_types = conf.get_recipe('state')
            setp_names, setp_types = conf.get_recipe('setp') 
            watchdog_names, watchdog_types = conf.get_recipe('watchdog')
            
            self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT_1)
            self.con.connect()
            self.con.get_controller_version()
            self.con.send_output_setup(state_names, state_types)
            self.setp = self.con.send_input_setup(setp_names, setp_types)
            self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)
            
            if not self.con.send_start():
                print('Failed to start data synchronization')
                sys.exit()
            
            for i in range(6):
                self.setp.__setattr__(f'input_double_register_{i}', 0)
            
            self.ui.outputResponse.append(f' [INFO]\tRTDE connection setup successfully.')
            
            # ------------ Initialize dashboard server connection ------------
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((ROBOT_HOST, ROBOT_PORT_2))
            polyscope_ver = self.s.send(('PolyscopeVersion' + '\n').encode())
            software_ver = self.s.send(('Version' + '\n').encode())
            self.ui.outputResponse.append(f' [INFO]\tDashboard server connection setup successfully.')
            
            self.ui.outputResponse.append(f'\n Successfully connected to {ROBOT_HOST}')
            self.ui.outputResponse.append(f' - Polyscope Version\t: {float(polyscope_ver)}')
            self.ui.outputResponse.append(f' - Software Version\t: {float(software_ver)}\n')
            self.ui.connectionStatus.setChecked(True)
            self.ui.connectBtn.setEnabled(False)
            self.rt_con_status = True
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tConnection Error to {ROBOT_HOST}.')
    
    def end_connection(self):
        self.con.disconnect()
        self.s.close()
        self.ui.connectionStatus.setChecked(False)
        self.ui.connectBtn.setEnabled(True)
        self.rt_con_status = False
    
    def check_connection(self):
        if self.rt_con_status:
            self.ui.connectionStatus.setChecked(True)
            print("[INFO] Status: Connected")
        else:
            self.ui.connectionStatus.setChecked(False)
            print("[INFO] Status: Not Connected")
    
    def power_on(self):
        try:
            self.s.send(('power on' + '\n').encode())
            self.ui.outputResponse.append(' [ACTION]\tButton "Power On" is triggered successfully.')
        except Exception as e:
            self.ui.outputResponse.append(' [ERROR]\tError triggering the selected button. Please check the remote connection.')
    
    def power_off(self):
        try:
            self.s.send(('power off' + '\n').encode())
            self.ui.outputResponse.append(' [ACTION]\tButton "Power Off" is triggered successfully.')
        except Exception as e:
            self.ui.outputResponse.append(' [ERROR]\tError triggering the selected button. Please check the remote connection.')
    
    def brake_release(self):
        try:
            self.s.send(('brake release' + '\n').encode())
            self.ui.outputResponse.append(' [ACTION]\tButton "Brake Release" is triggered successfully.')
        except Exception as e:
            self.ui.outputResponse.append(' [ERROR]\tError triggering the selected button. Please check the remote connection.')

    def update_speed(self, value):
        try:
            # Scale the slider value to the desired speed range
            scaled_speed = value / 100.0  # Assuming the speed range is from 0 to 1
            
            # Send the speed value to the robot using RTDE
            self.setp.speed_slider_mask = 1   # Assuming speed_slider_mask is the mask for speed control
            self.setp.speed_slider_fraction = scaled_speed
            
            self.ui.outputResponse.append(f' [INFO]\tSpeed set to: {scaled_speed}')
        except Exception as e:
            self.ui.outputResponse.append(' [ERROR]\tError setting the speed.')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = URCommunication_UI()
    ui.show()
    app.exec()