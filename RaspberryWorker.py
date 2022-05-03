from PyQt5.QtCore import QThread
import RPi.GPIO as GPIO
import serial
from time import sleep
from queue import Queue


class RaspberryWorker(QThread):
    # TODO gpio pin numbers to constants

    def __init__(self):
        super().__init__()
        self.queue = Queue()
        GPIO.setmode(GPIO.BCM)  # set pin numbering of the RPi board
        GPIO.setup(14, GPIO.OUT)  # DO - parameter SLOW
        GPIO.setup(15, GPIO.OUT)  # DO - parameter RUN
        GPIO.setup(16, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # DI - parameter COUNT

        GPIO.output(14, GPIO.HIGH)  # initial state for pin 14 - disable SLOW relay
        GPIO.output(15, GPIO.HIGH)  # initial state for pin 15 - disable RUN relay

        # serial line initialization
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # the last number of port depends on order of USBs connecting !!!
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1,
            xonxoff=False,  # disable software flow control
            rtscts=False,  # disable hardware flow control (RTS/CTS)
            dsrdtr=False  # disable hardware flow control (DSR/DTR)
        )

        self.serial_cleanup()

        self.big_cycles = 0
        self.particle_counter = 0
        self.caps_counter = 0

    def serial_cleanup(self):
        try:  # make sure the serial connection is clean and ready
            self.ser.open()  # opent the serial port with the parameters set above

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.flush()

            self.ser.close()  # close serial port
            self.ser.open()  # open serial port
        except Exception as ex:
            print(ex)

    def homing(self):
        if self.ser.isOpen():  # if the serial connection settings was successful, run the "try" block
            try:
                print("Homing the nozzle...")
                self.ser.write(b"!0WX2000,4000,100,500\r\n")  # calibrate x-axis
                sleep(4)
                self.ser.write(b"!0WY-2000,4000,100,500\r\n")  # calibrate y-axis
                sleep(2)
                self.ser.write(b"!0V3000\r\n")
                sleep(.001)
                self.ser.write(b"!0L0,740,0\r\n")
                sleep(.5)
                self.ser.write(b"!0L-1950,0,0\r\n")

                answer = self.ser.readline()
                print(answer)
                print("Homing successful, yeah!")
                sleep(1)

            except Exception as ex:
                print(ex)

    def particles_counting(self, particles, slow):
        GPIO.output(15, GPIO.LOW)  # ENABLES parameter RUN -> dispenser running
        # TODO to function
        while self.particle_counter < particles:
            if GPIO.input(16):  # Input pin controlling the beam
                sleep(.0006)
                self.particle_counter += 1
                print("Particles dispensed:", self.particle_counter)
                # TODO signal to update lcdNumber display
                # self.lcdNumber_Parts.display(self.counter)

            if self.particle_counter >= (particles - slow):
                GPIO.output(14, GPIO.LOW)  # ENABLES parameter SLOW DOWN -> dispenser running slower

    def dispensing(self, particles, slow):
        # TODO rename slow variable
        self.homing()
        try:
            for i in range(self.big_cycles):
                # TODO 9 to self.variable...
                for j in range(9):
                    try:
                        self.particles_counting(particles, slow)
                        self.particle_counter = 0  # reset counter
                        self.caps_counter += 1
                        # TODO signal to update progress bar

                        GPIO.output(15, GPIO.HIGH)
                        GPIO.output(14, GPIO.HIGH)
                        # TODO signal to update lcdNumber display
                        # self.lcdNumber_Caps.display(self.caps_counter)

                        sleep(2)
                        # TODO to function
                        if self.ser.isOpen():
                            try:
                                print("Moving the stage...")
                                self.ser.write(b"!0L-1061,0,0\r\n")
                                sleep(1)

                            except Exception as e:
                                print(e)

                    except Exception as e:
                        print(e)

                try:
                    self.particles_counting(particles, slow)
                    self.particle_counter = 0  # reset counter
                    self.caps_counter += 1

                    GPIO.output(15, GPIO.HIGH)
                    GPIO.output(14, GPIO.HIGH)
                    # TODO signal to update lcd and progress bar
                    # self.lcdNumber_Caps.display(self.caps_counter)
                    # self.progressBar.setValue(self.progress)

                    sleep(2)
                    if self.ser.isOpen():
                        try:
                            print("Moving the stage...")
                            self.ser.write(b"!0V5000\r\n")
                            self.ser.write(b"!0L9560,1070,0\r\n")

                        except Exception as e:
                            print(e)
                    sleep(3)

                except Exception as e:
                    print(e)

        except KeyboardInterrupt:  # KeyboardInterrupt (when ctrl+c pressed), exit the program
            print("Interrupted")
            GPIO.output(14, GPIO.HIGH)  # DISABLES parameter RUN
            GPIO.output(15, GPIO.HIGH)  # DISABLES parameter SLOW DOWN
            self.ser.write(
                b"!0L0,0,0\r\n")  # the exact numbers here might have to be adjusted according to the used nozzle

    def run(self):
        while True:
            task_to_do = self.queue.get()
            if "dispense" in task_to_do.keys():
                self.dispensing(particles=task_to_do["dispense"][0], slow=task_to_do["dispense"][1])

            sleep(0.1)
