# Look at QTimer only

import os
import sys
import logging
import socket
import sqlite3
import subprocess
import re
import time
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
from PyQt5.QtCore import Qt, QTimer, QThread, pyqtSignal, QEventLoop
from PyQt5.QtWidgets import QMainWindow, QApplication, QTableWidgetItem, QMessageBox
from PyQt5.QtGui import QIcon
from PyQt5.uic import loadUi
from main_ui import Ui_MainWindow


ROBOT_PORT_1 = 30004    # RTDE
ROBOT_PORT_2 = 29999    # Socket
config_filename = 'config/main-config.xml'
motion_database = 'data/motion-data.db'
UR_script = 'rtde_control_loop.urp'


class ConnectionThread(QThread):
    bg_con_status = pyqtSignal(bool)
    finished = pyqtSignal()

    def __init__(self, parent, con):
        super().__init__(parent)
        self.parent = parent
        self.con = con
        self.con_status = True

    def run(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_connection)
        self.timer.start(1000)  # Check connection every second

        loop = QEventLoop()
        loop.exec_()

    def check_connection(self):
        try:
            test = self.con.get_controller_version()
            if test == (None, None, None, None):
                self.parent.ui.connectionStatus.setChecked(False)
                self.con_status = False
            else:
                self.parent.ui.connectionStatus.setChecked(True)
                self.con_status = True

        except TimeoutError as e:
            self.parent.ui.connectionStatus.setChecked(False)
            self.con_status = False
            logging.error(f'RTDE connection error: {e}\nReconnecting....')

        except Exception as e:
            self.parent.ui.connectionStatus.setChecked(False)
            self.con_status = False
            print('RTDE still in connection state.')

        self.bg_con_status.emit(self.con_status)

    def stop(self):
        """Method to stop the worker_thread thread"""
        self.timer.stop()
        self.quit()
        self.wait()


class RobotControlThread(QThread):
    next_setp = pyqtSignal(list)
    progress = pyqtSignal(float)
    finished = pyqtSignal()
    error = pyqtSignal(str)

    def __init__(self, parent, setpoints, num_repetition, con, setp, watchdog):
        super().__init__(parent)
        self.parent = parent
        self.setpoints = setpoints
        self.num_repetition = num_repetition
        self.con = con
        self.setp = setp
        self.watchdog = watchdog
        self.running = True

    def run(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.move_robot)
        self.timer.start(500)  # Move robot every 500 milliseconds

        loop = QEventLoop()
        loop.exec_()

    def move_robot(self):
        num_setpoints = len(self.setpoints)
        movement_count = 0
        current_setpoint_index = -1
        move_completed = True
        refresh_rate = 6 - self.parent.ui.refreshRate.value()
        
        if not self.running:
            return

        if movement_count < self.num_repetition + (1 / num_setpoints):
            state = self.con.receive()
            if state is None:
                return

            if move_completed and state.output_int_register_0 == 1:
                if self.num_repetition - movement_count < 0.1 and self.parent.setp_to_list(
                        self.setp) != self.setpoints[0]:
                    last_setpoint = self.setpoints[0]
                    self.parent.list_to_setp(self.setp, last_setpoint)
                    self.con.send(self.setp)
                    self.watchdog.input_int_register_0 = 1
                    self.next_setp.emit(last_setpoint)
                    time.sleep(3)
                    return
                else:
                    move_completed = False
                    current_setpoint_index = (current_setpoint_index + 1) % num_setpoints
                    new_setpoint = self.setpoints[current_setpoint_index]
                    self.parent.list_to_setp(self.setp, new_setpoint)
                    self.con.send(self.setp)
                    self.watchdog.input_int_register_0 = 1
                    self.next_setp.emit(new_setpoint)

            elif not move_completed and state.output_int_register_0 == 0:
                move_completed = True
                self.watchdog.input_int_register_0 = 0
                movement_count += (1 / num_setpoints)
                self.progress.emit(movement_count)

            self.con.send(self.watchdog)

        else:
            self.timer.stop()
            time.sleep(5)

        if not self.running:
            return

    def stop(self):
        """Method to stop the worker thread"""
        self.timer.stop()
        self.quit()
        self.wait()



class URCommunication_UI(QMainWindow):
    def __init__(self):
        super(URCommunication_UI, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.connection_thread = None
        self.worker_thread = None

        self.load_adapter_dropdown()
        self.ui.serverInput.setText('192.168.189.129')
        self.ui.connectBtn.clicked.connect(self.setup_connection)
        self.ui.disconnectBtn.setEnabled(False)
        self.ui.disconnectBtn.clicked.connect(self.end_connection)

        self.ui.powerOnBtn.clicked.connect(self.power_on)
        self.ui.powerOffBtn.clicked.connect(self.power_off)
        self.ui.brakeReleaseBtn.clicked.connect(self.brake_release)
        
        self.ui.startFreeDriveBtn.setEnabled(False)
        self.ui.endFreeDriveBtn.setEnabled(False)
        self.ui.urpFileInput.setText(UR_script)
        self.ui.loadUrpBtn.clicked.connect(self.load_urp_file)
        self.ui.playBtn.clicked.connect(self.play_robot)
        self.ui.stopBtn.clicked.connect(self.stop_robot)
        self.ui.pauseBtn.clicked.connect(self.pause_robot)
        self.ui.shutDownBtn.clicked.connect(self.shutdown_robot)
        self.ui.shutDownBtn.setEnabled(False)
        self.ui.unlockBtn.clicked.connect(self.unlock_protective_stop)
        self.ui.closePopUpBtn.clicked.connect(self.close_safety_popup)
        
        self.ui.refreshBtn.clicked.connect(self.refresh_motion_dropdown)
        self.ui.loadMotionBtn.clicked.connect(self.load_position_data)
        self.ui.deleteBtn.clicked.connect(self.delete_motion_table)
        self.ui.recordBtn.clicked.connect(self.record_position)
        self.ui.runRobotBtn.clicked.connect(self.run_robot_command)
        self.ui.numRepetition.setValue(3)
        
        self.ui.clearOutputBtn.clicked.connect(self.ui.outputResponse.clear)
        self.ui.clearTableBtn.clicked.connect(self.clear_table)
    
    def get_ipv4_addresses(self):
        try:
            ipconfig_output = subprocess.check_output(["ipconfig", "/all"], universal_newlines=True)
            ipv4_pattern = r"IPv4 Address(?:.*): (\d+\.\d+\.\d+\.\d+)"
            matches = re.findall(ipv4_pattern, ipconfig_output)

            # Add matched IPv4 addresses to the list
            ipv4_addresses = []
            for match in matches:
                ipv4_addresses.append(match)
        
        except subprocess.CalledProcessError:
            print("Error running ipconfig command.")
        
        return ipv4_addresses
    
    def load_adapter_dropdown(self):
        ipv4_addresses = self.get_ipv4_addresses()
        self.ui.adapterSelect.clear()
        self.ui.adapterSelect.addItems(ipv4_addresses)
    
    def send_dashboard_server_command(self, command):
        try:
            self.s.send((command + '\n').encode())
            response = self.s.recv(1024).decode()
            return response
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}')
            return None
    
    def initialize_rtde_connection(self, ROBOT_HOST):
        try:
            logging.info('Initializing RTDE connection...')
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
                self.logger.error('Failed to start data synchronization')
                sys.exit()
            
            for i in range(6):
                self.setp.__setattr__(f'input_double_register_{i}', 0)
            
            self.watchdog.input_int_register_0 = 0
            
            self.ui.outputResponse.append(' [INFO]\tRTDE connection initialized successfully.')
            logging.info('RTDE connection initialized successfully.')
            return True
            
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError initializing RTDE connection: {e}.')
            logging.error(f'Error initializing RTDE connection: {e}.')
            return False

    def initialize_dashboard_server_connection(self):
        try:
            logging.info('Initializing dashboard server connection...')
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.settimeout(0.5)
            self.s.connect((self.ROBOT_HOST, ROBOT_PORT_2))
            self.s.recv(1024).decode()
            
            self.ui.outputResponse.append(' [INFO]\tDashboard server connection initialized successfully.')
            logging.info('Dashboard server connection initialized successfully.')
            return True
            
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError initializing dashboard server connection: {e}.')
            logging.error(f'Error initializing dashboard server connection: {e}.')
            return False

    def initialize_database_connection(self):
        try:
            logging.info('Initializing database connection...')
            self.conn = sqlite3.connect(motion_database)
            self.cur = self.conn.cursor()
            self.ui.outputResponse.append(' [INFO]\tDatabase connection initialized successfully.\n')
            logging.info('Database connection initialized successfully.')
            return True
            
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError initializing database connection: {e}.')
            logging.error(f'Error initializing database connection: {e}.')
            return False

    def setup_connection(self):
        self.ROBOT_HOST = str(self.ui.serverInput.text())
        
        rtde_connection_success = self.initialize_rtde_connection(self.ROBOT_HOST)
        dashboard_connection_success = self.initialize_dashboard_server_connection()
        database_connection_success = self.initialize_database_connection()
        
        if rtde_connection_success and dashboard_connection_success and database_connection_success:
            polyscope_ver = self.send_dashboard_server_command('PolyScopeVersion')
            serial_num = self.send_dashboard_server_command('get serial number')
            robot_model = self.send_dashboard_server_command('get robot model')
            
            self.ui.outputResponse.append(f' [INFO]\tSuccessfully connected to robot host "{self.ROBOT_HOST}".')
            self.ui.outputResponse.append(f' [INFO]\tPolyscope Version: {polyscope_ver[:-1]}')
            self.ui.outputResponse.append(f' [INFO]\tSerial Number: {serial_num[:-1]}')
            self.ui.outputResponse.append(f' [INFO]\tRobot Model: {robot_model[:-1]}\n')
            
            self.ui.connectionStatus.setChecked(True)
        else:
            self.ui.connectionStatus.setChecked(False)
            self.ui.outputResponse.append(f' [ERROR]\tFailed to establish connection to {self.ROBOT_HOST}.\n')
        
        if self.ui.bgProcess.isChecked() and self.connection_thread is None:
            self.connection_thread = ConnectionThread(self, self.con)
            self.connection_thread.bg_con_status.connect(self.handle_background_connection)
            self.connection_thread.finished.connect(self.handle_real_time_connection_finished)
            self.connection_thread.start()
        
        self.load_database_motion()
        self.ui.bgProcess.setEnabled(False)
        self.ui.shutDownBtn.setEnabled(False)
        self.ui.connectBtn.setEnabled(False)
        self.ui.disconnectBtn.setEnabled(True)
    
    def end_connection(self):
        self.con.connect()
        
        # Close all connections
        if hasattr(self, 'con'):
            self.con.disconnect()
        if hasattr(self, 's'):
            self.s.close()
        if hasattr(self, 'conn'):
            self.conn.commit()
            self.conn.close()
        
        # Stop the real-time connection thread
        try:
            if self.ui.bgProcess.isChecked() and self.connection_thread is not None:
                self.connection_thread.stop()
                self.connection_thread.wait()  # Wait for the thread to finish
                self.connection_thread = None  # Reset to None after thread finishes
        
        except Exception as e:
            pass
        
        # Stop the robot control thread
        if hasattr(self, 'worker_thread') and self.worker_thread is not None:
            self.worker_thread.stop()
            self.worker_thread.wait()
            self.worker_thread = None
        
        self.ui.bgProcess.setEnabled(True)
        self.ui.bgProcess.setChecked(False)
        self.ui.connectionStatus.setChecked(False)
        self.ui.connectBtn.setEnabled(True)
        self.ui.disconnectBtn.setEnabled(False)
        self.ui.outputResponse.append(' [INFO]\tServer disconnected.')

    
    def handle_background_connection(self, bg_connection):
        if not bg_connection:
            print('Reconnecting...')
            self.con.disconnect()
            self.initialize_rtde_connection(self.ui.serverInput.text())
        else:
            print('RTDE connection established.')
    
    def handle_real_time_connection_finished(self):
        self.connection_thread = None
        print('Real time connection background process discarded.')
        self.ui.outputResponse.append(' [INFO]\tReal time connection background process discarded.\n')
    
    def power_on(self):
        try:
            response = self.send_dashboard_server_command('power on')
            self.ui.outputResponse.append(' [INFO]\tButton "Power On" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def power_off(self):
        try:
            response = self.send_dashboard_server_command('power off')
            self.ui.outputResponse.append(' [INFO]\tButton "Power Off" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def brake_release(self):
        try:
            response = self.send_dashboard_server_command('brake release')
            self.ui.outputResponse.append(' [INFO]\tButton "Brake Release" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def load_urp_file(self):
        try:
            urp_file = str(self.ui.urpFileInput.text())
            load_urp_resp = self.send_dashboard_server_command(f'load {urp_file}')
            self.ui.outputResponse.append(f' [INFO]\t{load_urp_resp[:-1]}')
            
            if 'Loading' in load_urp_resp:
                response = self.send_dashboard_server_command('get loaded program')
                self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
            else:
                pass
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def play_robot(self):
        try:
            response = self.send_dashboard_server_command('play')
            self.ui.outputResponse.append(' [INFO]\tButton "Play Robot" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def stop_robot(self):
        try:
            response = self.send_dashboard_server_command('stop')
            self.ui.outputResponse.append(' [INFO]\tButton "Stop Robot" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
            self.ui.runRobotBtn.setEnabled(True)
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def pause_robot(self):
        try:
            response = self.send_dashboard_server_command('pause')
            self.ui.outputResponse.append(' [INFO]\tButton "Pause Robot" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def shutdown_robot(self):
        try:
            response = self.send_dashboard_server_command('shutdown')
            self.ui.outputResponse.append(' [INFO]\tButton "Shutdown Robot" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def unlock_protective_stop(self):
        try:
            response = self.send_dashboard_server_command('unlock protective stop')
            self.ui.outputResponse.append(' [INFO]\tButton "Unlock Protective Stop" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def close_safety_popup(self):
        try:
            response = self.send_dashboard_server_command('close safety popup')
            self.ui.outputResponse.append(' [INFO]\tButton "Close Safety Popup" is triggered successfully.')
            self.ui.outputResponse.append(f' [INFO]\t{response[:-1]}\n')
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
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
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
            return False
    
    def refresh_motion_dropdown(self):
        if not self.ui.runRobotBtn.isEnabled():
            self.stop_robot_command()
        elif self.load_database_motion():
            self.ui.outputResponse.append(f' [INFO]\tMotion type successfully refreshed.')
            self.ui.outputResponse.append(f' [INFO]\tAvailable Motions: {self.motions}\n')
        else:
            self.ui.outputResponse.append(f' [ERROR]\tError loading motion table.')
    
    def load_position_data(self):
        try:
            selected_motion = self.ui.motionSelect.currentText()
            self.cur.execute(f'''SELECT * FROM {selected_motion}''')
            motion_data = self.cur.fetchall()
            self.ui.outputResponse.append(f' [INFO]\tMotion "{selected_motion}" successfully loaded.')
            self.ui.motionTable.clearContents()
            self.ui.progressBar.setValue(0)
            self.ui.recordBtn.setEnabled(False)
            
            self.setpoints = []
            for row in motion_data:
                # Change data type from tuple to list
                setpoint = list(row[1:])
                self.setpoints.append(setpoint)
                self.ui.outputResponse.append(f' [INFO]\tSetpoint {str(row[0])}: {str(setpoint)}')
            
            self.ui.outputResponse.append('')
            
            # Arrange the position data into QTableWidget
            for row_index, row_data in enumerate(self.setpoints):
                self.ui.motionTable.insertRow(row_index)
                for col_index, col_value in enumerate(row_data):
                    self.ui.motionTable.setItem(row_index, col_index, QTableWidgetItem(str(f'{col_value}')))
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def delete_motion_table(self):
        selected_motion = self.ui.motionSelect.currentText()
        try:
            if selected_motion:
                confirm = QMessageBox.warning(self, 'Confirm Deletion - UR Communication Tool', f'Deleting "{selected_motion}" will erase all the positional data in it.\nDo you want to continue?',
                                            QMessageBox.Yes | QMessageBox.No)
                if confirm == QMessageBox.Yes:
                    self.cur.execute(f'DROP TABLE IF EXISTS {selected_motion}')
                    self.conn.commit()
                    self.ui.outputResponse.append(f' [INFO]\tMotion data "{selected_motion}" deleted successfully.\n')
                    self.load_database_motion()
                    self.ui.motionTable.clearContents()
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError deleting data: {e}.\n')
        
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
            self.ui.outputResponse.append(f' [INFO]\tPositions recorded and saved to database as "{self.new_table}".')
            self.ui.outputResponse.append(f' [INFO]\t{str(current_pos)}\n')
            self.load_database_motion()
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
            self.con.disconnect()
            self.initialize_rtde_connection(self.ROBOT_HOST)
    
    @staticmethod
    def setp_to_list(sp):
        sp_list = []
        for i in range(0, 6):
            sp_list.append(sp.__dict__["input_double_register_%i" % i])
        return sp_list
    
    @staticmethod
    def list_to_setp(sp, list):
        for i in range(0, 6):
            sp.__dict__["input_double_register_%i" % i] = list[i]
        return sp
    
    def handle_next_setpoint(self, next_setpoint):
        self.ui.outputResponse.append(f' [INFO]\tNext Setpoint: {next_setpoint}\n')
    
    def handle_progress_update(self, movement_count):
        total_repetition = self.ui.numRepetition.value()
        progress_percentage = (movement_count/total_repetition) * 100
        self.ui.progressBar.setValue(int(progress_percentage))
        self.ui.outputResponse.append(f' [INFO]\tCurrent Progress: {movement_count:.2f} / {self.ui.numRepetition.value():.2f}')
    
    def handle_robot_movement_finished(self):
        selected_motion = self.ui.motionSelect.currentText()
        self.ui.runRobotBtn.setEnabled(True)
        self.ui.outputResponse.append(f' [INFO]\tRobot movement for "{selected_motion}" completed.\n')
    
    def handle_thread_error(self, error_signal):
        self.ui.outputResponse.append(error_signal)
    
    def run_robot_command(self):
        try:
            self.worker_thread = RobotControlThread(self, self.setpoints, self.ui.numRepetition.value(), self.con, self.setp, self.watchdog)
            self.worker_thread.next_setp.connect(self.handle_next_setpoint)
            self.worker_thread.progress.connect(self.handle_progress_update)
            self.worker_thread.finished.connect(self.handle_robot_movement_finished)
            self.worker_thread.error.connect(self.handle_thread_error)
            self.worker_thread.start()
            self.ui.runRobotBtn.setEnabled(False)
        
        except Exception as e:
            self.ui.outputResponse.append(f' [ERROR]\tError receiving data: {e}.\n')
    
    def stop_robot_command(self):
        if hasattr(self, 'worker_thread') and self.worker_thread.isRunning():
            self.worker_thread.stop()
            self.worker_thread.wait() 
            self.worker_thread = None
            self.ui.outputResponse.append(' [INFO]\tRobot control thread stopped.')
        else:
            self.ui.outputResponse.append(' [INFO]\tNo running robot control thread.\n')
    
    def clear_table(self):
        self.ui.motionTable.clearContents()
        self.ui.recordBtn.setEnabled(True)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ui = URCommunication_UI()
    ui.show()
    app.exec()
