import sys
import logging
import socket
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget
from PyQt5.uic import loadUi

class URCommunication_UI(QMainWindow):
    def __init__(self):
        super(URCommunication_UI, self).__init__()
        loadUi('main.ui', self)
        
        self.serverInput.setText('192.168.189.129')
        self.connectionStatus.setEnabled(False)
        self.connectBtn.clicked.connect(self.ur_init)
    
    def ur_init(self):
        ROBOT_HOST = str(self.serverInput.text())
        ROBOT_PORT_1 = 30004    # RTDE
        ROBOT_PORT_2 = 29999    # Socket
        config_filename = 'config/main-config.xml'
        
        try:
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
            
            print(f'[Info] RTDE Connection setup successfully')
            
            # ------------ Initialize socket connection ------------
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((ROBOT_HOST, ROBOT_PORT_2))
            s.send(('PolyscopeVersion' + '\n').encode())
            print(f'[Info] Socket Connection setup successfully')
            
            print(f'Successfully connected to {ROBOT_HOST}')
            self.connectionStatus.setChecked(True)
            self.connectBtn.setEnabled(False)
        
        except Exception as e:
            print('Connection Error')
            
            
            


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = URCommunication_UI()
    ui.show()
    app.exec()