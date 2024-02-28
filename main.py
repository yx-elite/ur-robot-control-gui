import sys
import logging
import socket
import sqlite3
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QMainWindow, QApplication, QTableWidgetItem
from PyQt5.uic import loadUi
from main_ui import Ui_MainWindow


ROBOT_PORT_1 = 30004    # RTDE
ROBOT_PORT_2 = 29999    # Socket
config_filename = 'config/main-config.xml'
motion_database = 'data/motion-data.db'

class URCommunication_UI(QMainWindow):
    def __init__(self):
        super(URCommunication_UI, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        self.ui.serverInput.setText('192.168.189.129')
        self.ROBOT_HOST = str(self.ui.serverInput.text())
        self.ui.connectBtn.clicked.connect(self.setup_connection)
        self.ui.disconnectBtn.setEnabled(False)
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
        self.ui.refreshBtn.clicked.connect(self.refresh_motion_dropdown)
        self.ui.loadMotionBtn.clicked.connect(self.load_position_data)
        self.ui.recordBtn.clicked.connect(self.record_position)
        self.ui.runRobotBtn.clicked.connect(self.run_robot)
        
        self.ui.clearOutputBtn.clicked.connect(self.ui.outputResponse.clear)
        self.ui.clearTableBtn.clicked.connect(self.clear_table)

    def setup_connection(self):
        try:
            # ------------ Initialize rtde connection ------------
            self.keep_running = True
            logging.getLogger().setLevel(logging.INFO)
            
            conf = rtde_config.ConfigFile(config_filename)
            state_names, state_types = conf.get_recipe('state')
            setp_names, setp_types = conf.get_recipe('setp') 
            watchdog_names, watchdog_types = conf.get_recipe('watchdog')
            
            self.con = rtde.RTDE(self.ROBOT_HOST, ROBOT_PORT_1)
            self.con.connect()
            self.con.get_controller_version()
            self.con.send_output_setup(state_names, state_types)
            self.setp = self.con.send_input_setup(setp_names, setp_types)
            self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)
            
            if not self.con.send_start():
                print('Failed to start data synchronization')
                sys.exit()
            
            # Initialize fields for input and output
            for i in range(6):
                self.setp.__setattr__(f'input_double_register_{i}', 0)
            
            self.watchdog.input_int_register_0 = 0
            
            self.ui.outputResponse.append(f' [INFO]\tRTDE connection initialized successfully.')
            
            # ------------ Initialize dashboard server connection ------------
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.ROBOT_HOST, ROBOT_PORT_2))
            polyscope_ver = self.s.send(('PolyscopeVersion' + '\n').encode())
            software_ver = self.s.send(('Version' + '\n').encode())
            self.ui.outputResponse.append(f' [INFO]\tDashboard server connection initialized successfully.')
            
            # ------------ Initialize dashboard server connection ------------
            self.conn = sqlite3.connect(motion_database)
            self.cur = self.conn.cursor()
            self.ui.outputResponse.append(f' [INFO]\tDatabase connection initialized successfully.')
            
            self.ui.outputResponse.append(f'\n Successfully connected to {self.ROBOT_HOST}')
            self.ui.outputResponse.append(f' - Polyscope Version\t: {float(polyscope_ver)}')
            self.ui.outputResponse.append(f' - Software Version\t: {float(software_ver)}\n')
            self.load_database_motion()
            self.ui.connectionStatus.setChecked(True)
            self.ui.connectBtn.setEnabled(False)
            self.ui.disconnectBtn.setEnabled(True)
            self.rt_con_status = True
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tConnection Error to {self.ROBOT_HOST}.')
        
    def end_connection(self):
        self.con.disconnect()
        self.s.close()
        self.conn.commit()
        self.conn.close()
        self.ui.connectionStatus.setChecked(False)
        self.ui.connectBtn.setEnabled(True)
        self.ui.disconnectBtn.setEnabled(False)
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
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def power_off(self):
        try:
            self.s.send(('power off' + '\n').encode())
            self.ui.outputResponse.append(' [ACTION]\tButton "Power Off" is triggered successfully.')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def brake_release(self):
        try:
            self.s.send(('brake release' + '\n').encode())
            self.ui.outputResponse.append(' [ACTION]\tButton "Brake Release" is triggered successfully.')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def create_database_table(self):
        # Create new motion table
        self.new_table = self.ui.motionNameInput.text()
        table_command = f'''CREATE TABLE IF NOT EXISTS {self.new_table} (
                            id INTEGER PRIMARY KEY,
                            x REAL,
                            y REAL,
                            z REAL,
                            rx REAL,
                            ry REAL,
                            rz REAL
                        )'''
        self.cur.execute(table_command)
    
    def load_database_motion(self):
        try:
            self.cur.execute("SELECT name FROM sqlite_master WHERE type='table';")
            self.motions = self.cur.fetchall()
            self.motions = [name[0] for name in self.motions]
            self.motions.sort()
            self.ui.motionSelect.clear()
            self.ui.motionSelect.addItems(self.motions)
            return True

        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
            return False
    
    def refresh_motion_dropdown(self):
        if self.load_database_motion():
            self.ui.outputResponse.append(f' [ACTION]\tMotion type successfully refreshed.')
            self.ui.outputResponse.append(f' [INFO]\t{self.motions}')
        else:
            self.ui.outputResponse.append(f' [ERROR]\tError loading motion table.')
    
    def load_position_data(self):
        try:
            selected_motion = self.ui.motionSelect.currentText()
            self.cur.execute(f'''SELECT * FROM {selected_motion}''')
            motion_data = self.cur.fetchall()
            self.ui.outputResponse.append(f' [ACTION]\tMotion "{selected_motion}" successfully loaded.')
            self.ui.motionTable.clearContents()
            self.ui.recordBtn.setEnabled(False)
            
            self.setpoints = []
            for row in motion_data:
                # Change data type from tuple to list
                setpoint = list(row[1:])
                self.setpoints.append(setpoint)
                self.ui.outputResponse.append(f' [INFO]\tSetp {str(row[0])}: {str(setpoint)}')
            
            # Arrange the position data into QTableWidget
            for row_index, row_data in enumerate(self.setpoints):
                self.ui.motionTable.insertRow(row_index)
                for col_index, col_value in enumerate(row_data):
                    self.ui.motionTable.setItem(row_index, col_index, QTableWidgetItem(str(f'{col_value}')))
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
        
    def record_position(self):
        try:
            self.create_database_table()
            
            for _ in range(10):
                # Receive and save the robot position data
                state = self.con.receive()
            current_pos = state.actual_TCP_pose
            
            # Display robot position data in QTableWidget
            # Find the number of row above with values
            row_count = 0
            for row_index in range(self.ui.motionTable.rowCount()):
                item = self.ui.motionTable.item(row_index, 0)
                if item and item.text():
                    row_count += 1
                else:
                    break

            self.ui.motionTable.insertRow(row_count)
            for col_index, col_value in enumerate(current_pos):
                item = QTableWidgetItem(str(col_value))
                self.ui.motionTable.setItem(row_count, col_index, item)
            
            # Save the robot position data to database
            self.cur.execute(f'''INSERT INTO {self.new_table} (x, y, z, rx, ry, rz)
                                VALUES (?, ?, ?, ?, ?, ?)''', 
                                (current_pos[0], current_pos[1], current_pos[2],
                                current_pos[3], current_pos[4], current_pos[5]))
            self.conn.commit()
            self.ui.outputResponse.append(f' [ACTION]\tPositions recorded and saved to database as "{self.new_table}".')
            self.ui.outputResponse.append(f' [INFO]\t{str(current_pos)}')
            self.load_database_motion()
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def setp_to_list(sp):
        sp_list = []
        for i in range(0, 6):
            sp_list.append(sp.__dict__["input_double_register_%i" % i])
        return sp_list
    
    def list_to_setp(sp, list):
        for i in range(0, 6):
            sp.__dict__["input_double_register_%i" % i] = list[i]
        return sp
    
    def run_robot(self):
        num_repetition = self.ui.numRepetition.value()
        num_setpoints = len(self.setpoints)
        movement_count = 0
        current_setpoint_index = -1
        move_complete = True
        keep_running = True
        
        while keep_running:
            if movement_count < num_repetition:
                state = self.con.recieve()
                if state is None: 
                    break
                
                if move_complete and state.output_int_register_0 == 1:
                    move_complete = False
                    current_setpoint_index = (current_setpoint_index + 1) % num_setpoints
                    new_setpoint = self.setpoints[current_setpoint_index]
                    
    
    def clear_table(self):
        self.ui.motionTable.clearContents()
        self.ui.recordBtn.setEnabled(True)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = URCommunication_UI()
    ui.show()
    app.exec()