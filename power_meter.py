import sys
import serial
import serial.tools.list_ports
from serial.tools.list_ports_common import ListPortInfo
from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from time import sleep
from threading import Thread, Lock, Timer
from queue import Queue
from os import path
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class UpdateInterval:
    SLOW = 1.0
    MEDIUM = 0.5
    FAST = 0.1

class PowerGraph(FigureCanvas):
    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super(PowerGraph, self).__init__(self.fig)
        self.setParent(parent)
        
        # Настройки графика
        self.axes.set_ylabel('Power (W)')
        self.axes.grid(True)
        self.axes.set_xticks([])  # Убираем цифры по оси X
        self.line, = self.axes.plot([], [], 'b-')
        
    def update_graph(self, history_watts):
        """Обновляет график с новыми данными мощности в ваттах"""
        x = range(len(history_watts))
        self.line.set_data(x, history_watts)
        self.axes.relim()
        self.axes.autoscale_view(True, True, True)
        self.draw()

class Model:
    def __init__(self):
        self._serial_connection = None
        self.captured_values = []
        self._captured_values_unit = "mW"
        self.data_update_interval = UpdateInterval.MEDIUM
        self._min_max_lock = Lock()
        self._current_power_lock = Lock()
        self._history_dbm = []  # История в dBm
        self._max_history_length = 100
        self.reset_min_max()
        self._reset_current_power()

    def _convert_dbm_to_watts(self, dbm):
        """Конвертирует dBm в ватты"""
        return 10 ** ((dbm - 30) / 10)

    def get_power_history_watts(self):
        """Возвращает историю мощности в ваттах"""
        return [self._convert_dbm_to_watts(dbm) for dbm in self._history_dbm]

    def get_com_ports(self):
        return serial.tools.list_ports.comports()
    
    def is_connected(self):
        return self._serial_connection is not None and self._serial_connection.is_open
    
    def get_min_max_power(self):
        with self._min_max_lock:
            return (self.min_power_dbm, self.min_power, self.min_power_unit,), (self.max_power_dbm, self.max_power, self.max_power_unit,)
        
    def get_current_power(self):
        with self._current_power_lock:
            return self.current_power_dbm, self.current_power, self.current_power_unit
        
    def set_data_update_interval(self, interval):
        self.data_update_interval = interval

    def get_data_update_interval(self):
        return self.data_update_interval
    
    def get_capture_count(self):
        return len(self.captured_values)
    
    def _reset_current_power(self):
        with self._current_power_lock:
            self.current_power_dbm = 0.0
            self.current_power = 0.0
            self.current_power_unit = "uW"

    def reset_min_max(self):
        with self._min_max_lock:
            self.max_power_dbm = -70.0
            self.min_power_dbm = 70.0
            self.max_power = 0.0
            self.min_power = 0.0
            self.max_power_unit = "uW"
            self.min_power_unit = "uW"

    def add_capture(self, current, min, max):
        current_power_dbm, current_power, current_power_unit = current
        min_power_dbm, min_power, min_power_unit = min
        max_power_dbm, max_power, max_power_unit = max
        current_multiply_factor = self._get_power_multiply_factor(current_power_unit, self._captured_values_unit)
        min_multiply_factor = self._get_power_multiply_factor(min_power_unit, self._captured_values_unit)
        max_multiply_factor = self._get_power_multiply_factor(max_power_unit, self._captured_values_unit)
        current_power *= current_multiply_factor
        min_power *= min_multiply_factor
        max_power *= max_multiply_factor
        self.captured_values.append(
            [
                [current_power_dbm, current_power, self._captured_values_unit], 
                [min_power_dbm, min_power, self._captured_values_unit], 
                [max_power_dbm, max_power, self._captured_values_unit]
            ]
        )
        return len(self.captured_values) - 1

    def get_capture(self, index):
        if index < 0 or index >= len(self.captured_values):
            return None
        return self.captured_values[index]
    
    def clear_captured(self):
        self.captured_values = []
    
    def connect(self, port):
        try:
            self._serial_connection = serial.Serial(port, 460800, timeout=3)
            self._history_dbm = []
        except:
            return False
        return True
    
    def disconnect(self):
        if self._serial_connection is not None:
            self._serial_connection.close()
            self._serial_connection = None
    
    def request_settings(self):
        self.set_data_update_interval(UpdateInterval.FAST)
        try:
            self._serial_connection.reset_input_buffer()
            self._serial_connection.write(b"Read\r\n")
            self._serial_connection.flush()
        except:
            return False
        return True

    def set_settings(self, frequency, attenuation):
        try:
            self._serial_connection.write(f"A{frequency:04d}+{attenuation:.2f}\r\n".encode("utf-8"))
            self._serial_connection.flush()
        except:
            return False
        return True

    def get_next_captured_power_unit(self):
        if self._captured_values_unit == "uW":
            return "mW"
        elif self._captured_values_unit == "mW":
            return "W"
        elif self._captured_values_unit == "W":
            return "uW"
        return "uW"
    
    def _get_power_multiply_factor(self, prev_unit, new_unit):
        multiply_factor = 1.0
        if new_unit == "uW":
            multiply_factor = 1000.0
        elif new_unit == "mW":
            multiply_factor = 1.0
        elif new_unit == "W":
            multiply_factor = 0.001

        if prev_unit == "uW":
            multiply_factor *= 0.001
        elif prev_unit == "mW":
            multiply_factor *= 1.0
        elif prev_unit == "W":
            multiply_factor *= 1000.0
        return multiply_factor

    def update_captured_power_unit(self, new_unit):
        multiply_factor = self._get_power_multiply_factor(self._captured_values_unit, new_unit)
        for i, value_list in enumerate(self.captured_values):
            for j in range(len(value_list)):
                self.captured_values[i][j][1] = self.captured_values[i][j][1] * multiply_factor
                self.captured_values[i][j][2] = new_unit
        self._captured_values_unit = new_unit

    def export_captured(self, file_name):
        unit = self._captured_values_unit
        try:
            with open(file_name, 'w') as f:
                f.write(f"Current (dBm),Current ({unit}),Min (dBm),Min ({unit}),Max (dBm),Max ({unit})\n")
                out = []
                for value_list in self.captured_values:
                    for category_list in value_list:
                        out.append(f"{category_list[0]:.2f}")
                        out.append(f"{category_list[1]:.2f}")
                f.write(",".join(out) + "\n")
        except:
            return False
        return True
        
    def read_and_parse(self):
        raw_data = self._serial_connection.read_until(b"A").decode("utf-8")
        if len(raw_data) == 0:
            return None, None, None
        power_sum = 0.0
        power_dbm_sum = 0.0
        if raw_data[0] == 'a' and len(raw_data) >= 5000:
            valid = 0
            power_unit = None
            power_sign = None
            for i in range(500):
                measurement = raw_data[i * 10 + 1:11 + i * 10]
                if len(measurement) != 10 or not measurement[0] in ['-', '+'] or not measurement[9] in ['u', 'm', 'w'] or not measurement[1:9].isdigit():
                    continue
                valid += 1
                if power_unit is None:
                    if measurement[9] == 'u':
                        power_unit = "uW"
                    elif measurement[9] == 'm':
                        power_unit = "mW"
                    elif measurement[9] == 'w':
                        power_unit = "W"
                if power_sign is None:
                    power_sign = measurement[0]
                power_dbm_sum += float(measurement[1:4]) / 10.0
                power_sum += float(measurement[4:9]) / 100.0
            with self._current_power_lock:
                if valid == 0:
                    self.current_power_dbm = 60.0
                    self.current_power = 999.99
                    self.current_power_unit = "W"
                else:
                    self.current_power_dbm = float(power_sign + str(power_dbm_sum / float(valid)))
                    self.current_power = power_sum / float(valid)
                    self.current_power_unit = power_unit
                    
                    self._history_dbm.append(self.current_power_dbm)
                    if len(self._history_dbm) > self._max_history_length:
                        self._history_dbm.pop(0)
                        
            with self._min_max_lock:
                if self.current_power_dbm > self.max_power_dbm:
                    self.max_power_dbm = self.current_power_dbm
                    self.max_power = self.current_power
                    self.max_power_unit = self.current_power_unit
                if self.current_power_dbm < self.min_power_dbm:
                    self.min_power_dbm = self.current_power_dbm
                    self.min_power = self.current_power
                    self.min_power_unit = self.current_power_unit
            return self.current_power_dbm, self.current_power, self.current_power_unit
        elif raw_data[0] == 'R' and len(raw_data) >= 10:
            separator = "+"
            if "-" in raw_data:
                separator = "-"
            try:
                sep_index = raw_data.index(separator)
                frequency = int(raw_data[1:sep_index])
                attenuation_dbm = float(raw_data[sep_index:10])
                return frequency, attenuation_dbm, None
            except:
                pass
        return None, None, None

class UpdateQueueWatcher(QObject):
    update_available = pyqtSignal(tuple)

    def __init__(self, queue):
        super(UpdateQueueWatcher, self).__init__()
        self.queue = queue

    def run(self):
        while not self.thread().isInterruptionRequested():
            self.update_available.emit(self.queue.get())

    def stop(self):
        self.thread().requestInterruption()
        self.thread().quit()
        self.thread().wait()

class View:
    def __init__(self, controller):
        self._controller = controller
        self._window = uic.loadUi(path.abspath(path.join(path.dirname(__file__), 'powerMeter.ui')))
        self._update_queue = Queue()
        
        # Сохраняем исходный centralWidget
        original_central = self._window.centralWidget()
        original_central.setParent(None)
        
        # Создаем новый главный контейнер
        main_widget = QtWidgets.QWidget()
        main_layout = QtWidgets.QHBoxLayout(main_widget)
        main_layout.setContentsMargins(5, 5, 5, 5)
        
        # Левая панель - исходный интерфейс
        left_widget = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        # Переносим все элементы
        for i in reversed(range(original_central.layout().count())):
            item = original_central.layout().takeAt(i)
            if item.widget():
                left_layout.addWidget(item.widget())
        
        # Правая панель - график
        right_widget = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        self.graph = PowerGraph(right_widget, width=6, height=4)  # Увеличиваем начальный размер
        
        # Добавляем минимальный размер для правой панели
        right_widget.setMinimumWidth(400)
        right_layout.addWidget(self.graph)
        
        # Добавляем обе панели
        main_layout.addWidget(left_widget, stretch=1)
        main_layout.addWidget(right_widget, stretch=2)
        
        self._window.setCentralWidget(main_widget)
        
        # Устанавливаем минимальный размер окна
        self._window.setMinimumSize(800, 600)
        
        # Принудительно обновляем геометрию
        QtWidgets.QApplication.processEvents()
        
        self._init_UI()
        self._start_update_watcher()
        

    def _init_UI(self):
        self._window.connectBtn.clicked.connect(self._controller.connect_btn_handler)
        self._window.pauseBtn.clicked.connect(self._controller.pause_btn_handler)
        self._window.resetStatsBtn.clicked.connect(self._controller.reset_stats_btn_handler)
        self._window.resetCapturedBtn.clicked.connect(self._controller.reset_captured_btn_handler)
        self._window.captureBtn.clicked.connect(self._controller.capture_btn_handler)
        self._window.exportBtn.clicked.connect(self._controller.export_btn_handler)
        self._window.updateSpeed.currentIndexChanged.connect(self._controller.update_speed_changed_handler)
        self._window.applySettingsBtn.clicked.connect(self._controller.apply_settings_btn_handler)
        self._window.readSettingsBtn.clicked.connect(self._controller.read_settings_btn_handler)
        self._window.capturedValues.horizontalHeader().sectionClicked.connect(self._controller.table_header_clicked_handler)
        self._window.frequency.wheelEvent = lambda event: None
        self._window.updateSpeed.wheelEvent = lambda event: None
        self._window.comPorts.wheelEvent = lambda event: None

    def show_UI(self):
        self._window.show()

    def set_connected(self, connected):
        self._window.connectBtn.setText("Disconnect" if connected else "Connect")
        self._window.comPorts.setEnabled(not connected)
        self._window.resetStatsBtn.setEnabled(connected)
        self._window.pauseBtn.setEnabled(connected)
        self._window.updateSpeed.setEnabled(connected)
        self._window.updateSpeedLabel.setEnabled(connected)
        self._window.attenuationLabel.setEnabled(connected)
        self._window.attenuation.setEnabled(connected)
        self._window.frequencyLabel.setEnabled(connected)
        self._window.frequency.setEnabled(connected)
        self._window.applySettingsBtn.setEnabled(connected)
        self._window.readSettingsBtn.setEnabled(connected)

    def set_ports_available(self, ports):
        self._window.comPorts.clear()
        self._window.comPorts.addItems(ports)
        self._window.connectBtn.setEnabled(len(ports) > 0)

    def set_power_data(self, current, min, max):
        current_dbm, current_w, current_unit = current
        min_dbm, min_w, min_unit = min
        max_dbm, max_w, max_unit = max
        if current_dbm is not None and current_w is not None and current_unit is not None:
            self._window.curDbm.setText(f"{current_dbm:.2f} dBm")
            self._window.curW.setText(f"{current_w:.2f} {current_unit}")
        if min_dbm is not None and min_w is not None and min_unit is not None:
            self._window.minDbm.setText(f"{min_dbm:.2f} dBm")
            self._window.minW.setText(f"{min_w:.2f} {min_unit}")
        if max_dbm is not None and max_w is not None and max_unit is not None:
            self._window.maxDbm.setText(f"{max_dbm:.2f} dBm")
            self._window.maxW.setText(f"{max_w:.2f} {max_unit}")

    def update_graph(self, history_watts):
        self.graph.update_graph(history_watts)

    def set_settings(self, frequency, attenuation):
        self._window.frequency.setValue(frequency)
        self._window.attenuation.setValue(attenuation)

    def resume_updates(self, resume):
        self._window.pauseBtn.setText("Pause updates" if resume else "Resume updates")
        self._window.readSettingsBtn.setEnabled(resume)

    def notify(self, text, duration=0):
        self._window.statusBar.showMessage(text, duration)

    def get_selected_port(self):
        return self._window.comPorts.currentText().split(":")[0]
    
    def get_update_speed_index(self):
        return self._window.updateSpeed.currentIndex()
    
    def get_settings(self):
        return self._window.frequency.value(), self._window.attenuation.value()
    
    def set_captured_power_unit(self, unit):
        for i in [1, 3, 5]:
            header = self._window.capturedValues.horizontalHeaderItem(i)
            header.setText(header.text().split(" ")[0] + " " + unit)

    def get_save_file_name(self):
        options = QtWidgets.QFileDialog.Options()
        options |= QtWidgets.QFileDialog.DontUseNativeDialog
        return QtWidgets.QFileDialog.getSaveFileName(None, "Save File", "", "CSV (*.csv);;All files (*)", options = options)

    def add_capture(self, current, min, max):
        current_dbm, current_w, _ = current
        min_dbm, min_w, _ = min
        max_dbm, max_w, _ = max
        row = self._window.capturedValues.rowCount()
        self._window.capturedValues.insertRow(row)
        self._window.capturedValues.setItem(row, 0, QtWidgets.QTableWidgetItem(f"{current_dbm:.2f}"))
        self._window.capturedValues.setItem(row, 1, QtWidgets.QTableWidgetItem(f"{current_w:.2f}"))
        self._window.capturedValues.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{min_dbm:.2f}"))
        self._window.capturedValues.setItem(row, 3, QtWidgets.QTableWidgetItem(f"{min_w:.2f}"))
        self._window.capturedValues.setItem(row, 4, QtWidgets.QTableWidgetItem(f"{max_dbm:.2f}"))
        self._window.capturedValues.setItem(row, 5, QtWidgets.QTableWidgetItem(f"{max_w:.2f}"))
        self.reset_export_btn_enabled(True)

    def reset_capture_table(self, is_connected=True):
        self._window.capturedValues.setRowCount(0)

    def enable_capture_section(self, enable):
        self._window.captureBtn.setEnabled(enable)
        self._window.capturedValues.setEnabled(enable)

    def reset_export_btn_enabled(self, enable):
        self._window.resetCapturedBtn.setEnabled(enable)
        self._window.exportBtn.setEnabled(enable)

    def _start_update_watcher(self):
        self._update_watcher_thread = QThread()
        self._update_watcher = UpdateQueueWatcher(self._update_queue)
        self._update_watcher.moveToThread(self._update_watcher_thread)
        self._update_watcher_thread.started.connect(self._update_watcher.run)
        self._update_watcher.update_available.connect(self._update_queue_callback)
        self._update_watcher_thread.start()

    def _update_queue_callback(self, update):
        func, args = update
        func(*args)

    def stop_update_watcher(self):
        self.add_update(lambda: None)
        self._update_watcher.stop()

    def add_update(self, func, *args):
        self._update_queue.put((func, args))

class Controller:
    def __init__(self):
        self._app = QtWidgets.QApplication(sys.argv)
        self._model = Model()
        self._view = View(self)
        self._com_port_update_stop = False
        self._com_port_update_thread = None
        self._data_processing_stop = False
        self._data_processing_thread = None
        self._start_com_port_update()

    def start_app(self):
        self._view.show_UI()
        self._app.exec()
        self.safe_exit()

    def safe_exit(self):
        self._stop_com_port_update()
        self._stop_data_processing()
        self._model.disconnect()
        self._view.stop_update_watcher()

    def connect_btn_handler(self):
        if not self._model.is_connected():
            port = self._view.get_selected_port()
            if self._model.connect(port):
                self._stop_com_port_update()
                self._start_data_processing()
                if not self._model.request_settings():
                    self.connect_btn_handler()
                    return
                Timer(0.5, self._model.request_settings).start()
                self._view.notify(f"Connected to {port}", 3000)
                self._view.set_connected(True)
                self._view.enable_capture_section(True)
            else:
                self._view.notify(f"Connection failed, try re-connecting the device", 3000)
        else:
            self._stop_data_processing()
            self._model.disconnect()
            self._start_com_port_update()
            if self._model.get_capture_count() == 0:
                self._view.enable_capture_section(False)
                self._view.reset_export_btn_enabled(False)
            self._view.set_connected(False)

    def pause_btn_handler(self):
        if self._data_processing_thread is None:
            self._start_data_processing()
            self._view.resume_updates(True)
            self._view.notify("Updates resumed!", 3000)
        else:
            self._stop_data_processing()
            self._view.resume_updates(False)
            self._view.notify("Updates paused")

    def reset_stats_btn_handler(self):
        self._model.reset_min_max()
        min_power, max_power = self._model.get_min_max_power()
        self._view.set_power_data((None, None, None), min_power, max_power)

    def reset_captured_btn_handler(self):
        self._model.clear_captured()
        self._view.reset_capture_table()
        self._view.reset_export_btn_enabled(False)
        if not self._model.is_connected():
            self._view.enable_capture_section(False)

    def capture_btn_handler(self):
        current_power = self._model.get_current_power()
        min_power, max_power = self._model.get_min_max_power()
        index = self._model.add_capture(current_power, min_power, max_power)
        current_power_converted, min_power_converted, max_power_converted = self._model.get_capture(index)
        self._view.add_capture(current_power_converted, min_power_converted, max_power_converted)
        self._view.reset_export_btn_enabled(True)

    def export_btn_handler(self):
        choice = self._view.get_save_file_name()
        file_name = choice[0]
        if len(file_name) == 0:
            return
        elif choice[1] == "CSV (*.csv)" and not file_name.endswith(".csv"):
            file_name = file_name + ".csv"
        
        if self._model.export_captured(file_name):
            self._view.notify(f"Values exported to {file_name}", 3000)
        else:
            self._view.notify(f"Export failed, please try again or in another location", 3000)

    def apply_settings_btn_handler(self):
        frequency, attenuation = self._view.get_settings()
        if not self._model.set_settings(frequency, attenuation):
            self.connect_btn_handler()
            return
        self._view.notify("Configuration applied!", 3000)

    def read_settings_btn_handler(self):
        self._model.request_settings()

    def update_speed_changed_handler(self, index, notify=True):
        if index == 0:
            self._model.set_data_update_interval(UpdateInterval.SLOW)
        elif index == 1:
            self._model.set_data_update_interval(UpdateInterval.MEDIUM)
        elif index == 2:
            self._model.set_data_update_interval(UpdateInterval.FAST)
        if notify:
            self._view.notify(f"Refresh rate set to {self._model.get_data_update_interval()}s", 3000)

    def table_header_clicked_handler(self, index):
        if not index in [1, 3, 5]:
            return
        new_unit = self._model.get_next_captured_power_unit()
        self._view.set_captured_power_unit(new_unit)
        self._model.update_captured_power_unit(new_unit)
        self._view.reset_capture_table()
        for i in range(self._model.get_capture_count()):
            current_power_converted, min_power_converted, max_power_converted = self._model.get_capture(i)
            self._view.add_capture(current_power_converted, min_power_converted, max_power_converted)

    def _update_com_ports(self):
        prev_ports = None
        while not self._com_port_update_stop:
            com_ports = self._model.get_com_ports()
            if prev_ports is None or len(com_ports) != len(prev_ports):
                prev_ports = com_ports
                self._view.add_update(self._view.set_ports_available, [f"{com_port.device}: {com_port.description}" for com_port in com_ports])
                self._view.add_update(self._view.notify, f"{len(com_ports)} serial devices available")
            sleep(0.5)

    def _stop_com_port_update(self):
        if self._com_port_update_thread is not None and self._com_port_update_thread.is_alive():
            self._com_port_update_stop = True
            self._com_port_update_thread.join()
            self._com_port_update_thread = None

    def _start_com_port_update(self):
        if self._com_port_update_thread is None or not self._com_port_update_thread.is_alive():
            self._com_port_update_stop = False
            self._com_port_update_thread = Thread(target=self._update_com_ports)
            self._com_port_update_thread.start()

    def _process_data(self):
        while not self._data_processing_stop:
            data = None
            try:
                data = self._model.read_and_parse()
            except:
                self._model.disconnect()
                self._view.add_update(self._view.set_connected, False)
                if self._model.get_capture_count() == 0:
                    self._view.add_update(self._view.enable_capture_section, False)
                    self._view.add_update(self._view.reset_export_btn_enabled, False)
                self._start_com_port_update()
                self._data_processing_stop = True
                continue
            if data[0] is not None:
                if data[2] is not None:
                    min_power, max_power = self._model.get_min_max_power()
                    self._view.add_update(self._view.set_power_data, data, min_power, max_power)
                    history_watts = self._model.get_power_history_watts()
                    self._view.add_update(self._view.update_graph, history_watts)
                else:
                    frequency, attenuation, _ = data
                    self._view.add_update(self._view.set_settings, frequency, attenuation)
                    self._view.add_update(self._view.notify, "Configuration fetched!", 3000)
                    self.update_speed_changed_handler(self._view.get_update_speed_index(), False)
            sleep(self._model.get_data_update_interval())

    def _stop_data_processing(self):
        if self._data_processing_thread is not None and self._data_processing_thread.is_alive():
            self._data_processing_stop = True
            self._data_processing_thread.join()
            self._data_processing_thread = None

    def _start_data_processing(self):
        if self._data_processing_thread is None or not self._data_processing_thread.is_alive():
            self._data_processing_stop = False
            self._data_processing_thread = Thread(target=self._process_data)
            self._data_processing_thread.start()

if __name__ == "__main__":
    controller = Controller()
    controller.start_app()
