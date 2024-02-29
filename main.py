import sys
import logging
import socket
import sqlite3
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import QMainWindow, QApplication, QTableWidgetItem, QMessageBox
from PyQt5.uic import loadUi
from main_ui import Ui_MainWindow


ROBOT_PORT_1 = 30004    # RTDE
ROBOT_PORT_2 = 29999    # Socket
config_filename = 'config/main-config.xml'
motion_database = 'data/motion-data.db'
UR_script = 'rtde_control_loop.urp'

class RobotWorkerThread(QThread):
    """
    A separate worker thread for robot controlling.
    
    Attributes:
        setpoints (list): List of setpoints for robot movements
        num_repetition (int): Number of repetitions for each setpoint
        con: Connection object for communication with the robot
        setp: Object representing setpoint data
        watchdog: Object representing watchdog status
    """
    track = pyqtSignal(list)
    progress = pyqtSignal(float)
    finished = pyqtSignal()
    
    def __init__(self, setpoints, num_repetition, con, setp, watchdog, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setpoints = setpoints
        self.num_repetition = num_repetition
        self.con = con
        self.setp = setp
        self.watchdog = watchdog
    
    def run(self):
        """Main method executed by the worker thread"""
        
        num_setpoints = len(self.setpoints)
        movement_count = 0
        current_setpoint_index = -1
        move_completed = True
        keep_running = True
        
        while keep_running:
            if movement_count < self.num_repetition:
                state = self.con.receive()
                if state is None:
                    break
                
                if move_completed and state.output_int_register_0 == 1:
                    move_completed = False
                    current_setpoint_index = (current_setpoint_index + 1) % num_setpoints
                    new_setpoint = self.setpoints[current_setpoint_index]
                    self.parent.list_to_setp(self.setp, new_setpoint)
                    self.con.send(self.setp)
                    self.watchdog.input_int_register_0 = 1
                    print(new_setpoint)
                    
                
                elif not move_completed and state.output_int_register_0 == 0:
                    move_completed = True
                    self.watchdog.input_int_register_0 = 0
                    movement_count += (1/num_setpoints)
                    print(movement_count)
                    time.sleep(1)
                
                self.con.send(self.watchdog)
            
            else:
                time.sleep(5)
                break
        
        # Move back to first position before ending
        last_setpoint = self.setpoints[0]
        
        
        self.finished.emit()


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
        # self.rt_con_status = False
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.check_connection)
        # self.timer.start(1000)
        
        self.ui.powerOnBtn.clicked.connect(self.power_on)
        self.ui.powerOffBtn.clicked.connect(self.power_off)
        self.ui.brakeReleaseBtn.clicked.connect(self.brake_release)
        
        # self.ui.startFreeDriveBtn.clicked.connect(self.enable_free_drive)
        # self.ui.endFreeDriveBtn.clicked.connect(self.disable_free_drive)
        self.ui.urpFileInput.setText(UR_script)
        self.ui.loadUrpBtn.clicked.connect(self.load_urp_file)
        self.ui.playBtn.clicked.connect(self.play_robot)
        self.ui.stopBtn.clicked.connect(self.stop_robot)
        self.ui.pauseBtn.clicked.connect(self.pause_robot)
        self.ui.shutDownBtn.clicked.connect(self.shutdown_robot)
        self.ui.unlockBtn.clicked.connect(self.unlock_protective_stop)
        self.ui.closePopUpBtn.clicked.connect(self.close_safety_popup)
        
        self.ui.refreshBtn.clicked.connect(self.refresh_motion_dropdown)
        self.ui.loadMotionBtn.clicked.connect(self.load_position_data)
        self.ui.deleteBtn.clicked.connect(self.delete_motion_table)
        self.ui.recordBtn.clicked.connect(self.record_position)
        self.ui.runRobotBtn.clicked.connect(self.run_robot)
        self.ui.numRepetition.setValue(3)
        
        self.ui.clearOutputBtn.clicked.connect(self.ui.outputResponse.clear)
        self.ui.clearTableBtn.clicked.connect(self.clear_table)
    
    def send_dashboard_server_command(self, command):
        try:
            self.s.send((command + '\n').encode())
            response = self.s.recv(1024).decode()
            self.ui.outputResponse.append(f' [ACTION]\t{response[:-1]}')
            return response
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}')
            return None

    def setup_connection(self):
        try:
            logging.info('Initializing RTDE connection...')
            logging.info('Initializing dashboard server connection...')
            logging.info('Initializing database connection...')
            
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
                self.logger.error('Failed to start data synchronization')
                sys.exit()
            
            # Initialize fields for input and output
            for i in range(6):
                self.setp.__setattr__(f'input_double_register_{i}', 0)
            
            self.watchdog.input_int_register_0 = 0
            
            logging.info('RTDE connection initialized successfully.')
            self.ui.outputResponse.append(f' [INFO]\tRTDE connection initialized successfully.')
            
            # ------------ Initialize dashboard server connection ------------
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((self.ROBOT_HOST, ROBOT_PORT_2))
            self.s.recv(1024).decode()
            self.s.send(('PolyScopeVersion' + '\n').encode())
            polyscope_ver = self.s.recv(1024).decode()
            self.s.send(('get serial number' + '\n').encode())
            serial_num = self.s.recv(1024).decode()
            self.s.send(('get robot model' + '\n').encode())
            robot_model = self.s.recv(1024).decode()
            self.ui.outputResponse.append(f' [INFO]\tDashboard server connection initialized successfully.')
            
            # ------------ Initialize dashboard server connection ------------
            self.conn = sqlite3.connect(motion_database)
            self.cur = self.conn.cursor()
            self.ui.outputResponse.append(f' [INFO]\tDatabase connection initialized successfully.')
            
            # ------------ Overall status display --------------------
            self.ui.outputResponse.append(f'\n Successfully connected to robot host "{self.ROBOT_HOST}".')
            self.ui.outputResponse.append(f' - Polyscope Version\t: {str(polyscope_ver[:-1])}')
            self.ui.outputResponse.append(f' - Serial Number\t: {str(serial_num[:-1])}')
            self.ui.outputResponse.append(f' - UR Robot Model\t: {str(robot_model[:-1])}\n')
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
            logging.info("Data synchronization successful.")
        else:
            self.ui.connectionStatus.setChecked(False)
            logging.error("Data synchronization not established.")
    
    def power_on(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Power On" is triggered successfully.')
            self.send_dashboard_server_command('power on')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def power_off(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Power Off" is triggered successfully.')
            self.send_dashboard_server_command('power off')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def brake_release(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Brake Release" is triggered successfully.')
            self.send_dashboard_server_command('brake release')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def load_urp_file(self):
        try:
            urp_file = str(self.ui.urpFileInput.text())
            load_urp_resp = self.send_dashboard_server_command(f'load {urp_file}')
            
            if 'Loading' in load_urp_resp:
                self.send_dashboard_server_command('get loaded program')
            else:
                pass
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def play_robot(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Play Robot" is triggered successfully.')
            self.send_dashboard_server_command('play')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def stop_robot(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Stop Robot" is triggered successfully.')
            self.send_dashboard_server_command('stop')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def pause_robot(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Pause Robot" is triggered successfully.')
            self.send_dashboard_server_command('pause')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def shutdown_robot(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Shutdown Robot" is triggered successfully.')
            self.send_dashboard_server_command('shutdown')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def unlock_protective_stop(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Unlock Protective Stop" is triggered successfully.')
            self.send_dashboard_server_command('unlock protective stop')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.')
    
    def close_safety_popup(self):
        try:
            self.ui.outputResponse.append(' [ACTION]\tButton "Close Safety Popup" is triggered successfully.')
            self.send_dashboard_server_command('close safety popup')
        
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
    
    def delete_motion_table(self):
        try:
            selected_motion = self.ui.motionSelect.currentText()
            if selected_motion:
                confirm = QMessageBox.warning(self, 'Confirm Deletion - UR Communication Tool', f'Deleting "{selected_motion}" will erase all the positional data in it.\nDo you want to continue?',
                                            QMessageBox.Yes | QMessageBox.No)
                if confirm == QMessageBox.Yes:
                    self.cur.execute(f'DROP TABLE IF EXISTS {selected_motion}')
                    self.conn.commit()
                    self.ui.outputResponse.append(f' [ACTION]\tMotion data "{selected_motion}" deleted successfully.')
                    self.load_database_motion()
                    self.ui.motionTable.clearContents()
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError deleting data: {e}.')
        
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
    
    def setp_to_list(self, sp):
        sp_list = []
        for i in range(0, 6):
            sp_list.append(sp.__dict__["input_double_register_%i" % i])
        return sp_list
    
    def list_to_setp(self, sp, list):
        for i in range(0, 6):
            sp.__dict__["input_double_register_%i" % i] = list[i]
        return sp
    
    def handle_test(self, new_setpoint):
        self.ui.outputResponse.append(f'Next setp: {new_setpoint}')
    
    def handle_progress_update(self, movement_count):
        self.ui.outputResponse.append(f'progress {movement_count}')
    
    def handle_robot_movement_finished(self):
        self.ui.runRobotBtn.setEnabled(True)
        print("Robot movement completed.")
    
    def run_robot(self):
        self.worker = RobotWorkerThread(self.setpoints, self.ui.numRepetition.value(), self.con, self.setp, self.watchdog, self)
        #self.worker.track.connect(self.handle_test)
        #self.worker.progress.connect(self.handle_progress_update)
        self.worker.finished.connect(self.handle_robot_movement_finished)
        self.worker.start()
        self.ui.runRobotBtn.setEnabled(False)
    
    def clear_table(self):
        self.ui.motionTable.clearContents()
        self.ui.recordBtn.setEnabled(True)



if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = URCommunication_UI()
    ui.show()
    app.exec()