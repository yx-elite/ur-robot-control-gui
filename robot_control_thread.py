import time
import logging
from PyQt5.QtCore import QThread, pyqtSignal


class RobotControlThread(QThread):
    """
    Worker thread for robot controlling.
    """
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
        num_setpoints = len(self.setpoints)
        movement_count = 0
        current_setpoint_index = -1
        move_completed = True
        keep_running = True
        
        self.progress.emit(movement_count)
        
        try:
            while keep_running:
                # Use to kill the thread for stopping
                if not self.running:
                    break
                
                # Add additional setpoint value to trigger last position
                if movement_count < self.num_repetition + (1/num_setpoints):
                    state = self.con.receive()
                    if state is None:
                        break
                    
                    if move_completed and state.output_int_register_0 == 1:
                        if self.num_repetition - movement_count < 0.1 and self.parent.setp_to_list(self.setp) != self.setpoints[0]:
                            # Move back to first position before ending
                            last_setpoint = self.setpoints[0]
                            self.parent.list_to_setp(self.setp, last_setpoint)
                            self.con.send(self.setp)
                            self.watchdog.input_int_register_0 = 1
                            self.next_setp.emit(last_setpoint)
                            break
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
                        movement_count += (1/num_setpoints)
                        self.progress.emit(movement_count)
                    
                    self.con.send(self.watchdog)
                
                else:
                    time.sleep(5)
                    break
            
        except Exception as e:
            logging.error(f'Connection Reset Error: {e}')
            self.error.emit(f' [ERROR]\tConnection Reset Error: {e}\n [ERROR]\tPlease reset the connection.')
        
        self.finished.emit()
    
    def stop(self):
        """Method to stop the worker thread"""
        self.running = False