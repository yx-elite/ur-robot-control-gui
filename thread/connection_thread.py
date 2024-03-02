import time
import logging
from PyQt5.QtCore import QThread, pyqtSignal


class ConnectionThread(QThread):
    """
    Worker thread for real time connection status checking.
    """
    bg_con_status = pyqtSignal(bool)
    program_state = pyqtSignal(str)
    finished = pyqtSignal()
    
    def __init__(self, parent, con):
        super().__init__(parent)
        self.parent = parent
        self.con = con
        self.running = True
        self.con_status = True
        
    def run(self):
        while True:
            refresh_rate = 6 - self.parent.ui.refreshRate.value()
            program_state = self.parent.send_dashboard_server_command('programState')
            self.program_state.emit(program_state)
            
            if not self.running:
                break
            
            try:
                test = self.con.get_controller_version()
                if test == (None, None, None, None):
                    self.parent.ui.connectionStatus.setChecked(False)
                    self.con_status = False
                    time.sleep(refresh_rate)
                else:
                    self.parent.ui.connectionStatus.setChecked(True)
                    self.con_status = True
                    time.sleep(refresh_rate)
                
            except TimeoutError as e:
                self.parent.ui.connectionStatus.setChecked(False)
                self.con_status = False
                logging.error(f'RTDE connection error: {e}\nReconnecting....')
            
            except Exception as e:
                self.parent.ui.connectionStatus.setChecked(False)
                self.con_status = False
                print('RTDE still in connection state.')
            
            self.bg_con_status.emit(self.con_status)
        
        self.finished.emit()
    
    def stop(self):
        """Method to stop the worker_thread thread"""
        self.running = False
        self.quit()
        self.wait()
