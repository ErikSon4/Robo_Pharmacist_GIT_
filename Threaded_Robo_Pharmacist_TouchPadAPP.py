import sys
import serial
from time import sleep
import RPi.GPIO as GPIO
from RaspberryWorker import RaspberryWorker

from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget

from TouchPad import Ui_RoboPharmacist_API


class MainWindow(QWidget, Ui_RoboPharmacist_API):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.iconName = 'green_cross_icon.png'
        self.show()

        self.pushButton_STOP.clicked.connect(self.stop_button_clicked)
        self.pushButton_RUN.clicked.connect(self.run_button_clicked)
        self.pushButton_MANUAL.clicked.connect(self.manual_button_clicked)

        # Parameters set in GUI
        self.ID = self.lineEdit_ID.text()  # product-specific code
        self.speed = 4
        self.slow_speed = 2
        self.sensitivity = 2

        self.counter = 0  # initial particles counter value
        self.caps_counter = 0  # initial capsules counter value
        self.progress = 0  # initial value of the progress bar
        self.big_cycles = 0
        self.progress_step = 0

        self.capsules = 0  # desired number of capsules to be filled
        self.particles = 0  # desired number of particles to be dispensed
        self.slow = 0  # number "slower parts"

        self.raspi_worker = RaspberryWorker()
        self.raspi_worker.start()
        self.thread = {}

    def stop_button_clicked(self):
        GPIO.output(14, GPIO.HIGH)  # DISABLES parameter RUN
        GPIO.output(15, GPIO.HIGH)  # DISABLES parameter SLOW DOWN
        ser.write(b"!0L0,0,0\r\n")  # the exact numbers here might have to be adjusted according to the used nozzle
        QApplication.processEvents()

    def manual_button_clicked(self):
        self.thread[1] = RunThread(parent=None, index=1)
        self.thread[1].start()
        self.thread[1].TimerCounter.connect(self.stopwatch)

    def stopwatch(self, text):
        timetext = text

        self.label_Timer.setText(timetext)

    def run_button_clicked(self):
        #  DEFINE PROCESS VARIABLES ------------------------------------------------------------------------------------
        self.capsules = int(self.lineEdit_CapsNumber.text())  # set the desired number of capsules to be filled
        self.particles = int(self.lineEdit_Number.text())  # set the desired number of particles to be dispensed
        self.slow = int(self.lineEdit_SlowParticles.text())  # set the number "slower parts"

        self.progress_step = 100 / self.capsules

        # while self.cycleconst >= 10:
        #     self.cycleconst -= 10
        #     self.big_cycles += 1
        self.raspi_worker.big_cycles = self.capsules // 10
        #  SET-UP RPi GPIO BOARD ---------------------------------------------------------------------------------------
        # GPIO.setmode(GPIO.BCM)  # set pin numbering of the RPi board
        # GPIO.setup(14, GPIO.OUT)  # DO - parameter SLOW
        # GPIO.setup(15, GPIO.OUT)  # DO - parameter RUN
        # GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # DI - parameter COUNT
        #
        # GPIO.output(14, GPIO.HIGH)  # initial state for pin 14 - disable SLOW relay
        # GPIO.output(15, GPIO.HIGH)  # initial state for pin 15 - disable RUN relay

        # HOMING THE NOZZLE ---------------------------------------------------------------------
        # ser = serial.Serial(
        #     port='/dev/ttyUSB0',  # the last number of port depends on order of USBs connecting !!!
        #     baudrate=115200,
        #     parity=serial.PARITY_NONE,
        #     stopbits=serial.STOPBITS_ONE,
        #     bytesize=serial.EIGHTBITS,
        #     timeout=1,
        #     xonxoff=False,  # disable software flow control
        #     rtscts=False,  # disable hardware flow control (RTS/CTS)
        #     dsrdtr=False  # disable hardware flow control (DSR/DTR)
        # )
        #
        # ser.close()  # close the serial port which is open by default but with wrong parameters
        # ser.open()  # open the serial port with parameters set above

        # try:  # make sure the serial connection is clean and ready
        #     ser.open()  # opent the serial port with the parameters set above
        #
        #     ser.reset_input_buffer()
        #     ser.reset_output_buffer()
        #     ser.flush()
        #
        #     ser.close()  # close serial port
        #     ser.open()  # open serial port

        # except Exception as e:  # if an error occurs, print the error message
        #     print(e)

        # if ser.isOpen():  # if the serial connection settings was successful, run the "try" block
        #     try:
        #         print("Homing the nozzle...")
        #         ser.write(b"!0WX20000,4000,100,500\r\n")  # calibrate x-axis
        #         sleep(4)
        #         ser.write(b"!0WY-20000,4000,100,500\r\n")  # calibrate y-axis
        #         sleep(2)
        #         ser.write(b"!0V3000\r\n")
        #         sleep(.001)
        #         ser.write(b"!0L0,740,0\r\n")
        #         sleep(.5)
        #         ser.write(b"!0L-1950,0,0\r\n")
        #
        #         answer = ser.readline()
        #         print(answer)
        #         print("Homing successful, yeah!")
        #         sleep(1)
        #
        #     except Exception as e:
        #         print(e)

        # DISPENSING ---------------------------------------------------------------------
        self.raspi_worker.queue.put({"dispense": [self.particles, self.slow]})


class RunThread(QtCore.QThread):
    TimerCounter = QtCore.pyqtSignal(str)

    def __init__(self, parent=None, index=0):
        super(RunThread, self).__init__(parent)
        self.index = index
        self.is_running = True

    def run(self):
        print("Starting Thread...", self.index)

        sec2 = 0
        sec1 = 0
        min2 = 0
        min1 = 0
        timetext = '00:00:00'

        while True:
            sec2 += 1

            if sec2 > 9:
                sec1 += 1
                sec2 = 0

            if sec1 > 5:
                min2 += 1
                sec1 = 0

            if min2 > 9:
                min1 += 1
                min2 = 0

            timetext = timetext[:3] + str(min1) + str(min2) + ':' + str(sec1) + str(sec2)
            self.TimerCounter.emit(timetext)
            sleep(1)

    def stop(self):
        self.is_running = False
        self.terminate()


app = QApplication(sys.argv)
w = MainWindow()
app.exec_()






