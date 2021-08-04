from tkinter import *
from tkinter import messagebox
import serial
from threading import Thread
import time
import queue
import sys
from socket import *

# -----------------------------------------------------------------------
# The function is read the date and time from the label,
# check the errors and send it to the TTL port


def change_time_date():
    time_date = entry_ch_time_data.get()

    matchObj = re.match(  # regular expression, two digits number for every part
        r"^(\d\d?):(\d\d?):(\d\d?):(\d\d?):(\d\d?):(\d\d?):(\d\d?)$", time_date)
    ok = 1

    if matchObj:
        if not (0 <= int(matchObj.group(1)) <= 99):
            ok = 0
            messagebox.showinfo(
                'Wrong value', 'Wrong number of year.\n0 <= year <= 99')
        if not (1 <= int(matchObj.group(2)) <= 12):
            messagebox.showinfo(
                'Wrong value', 'Wrong number of month.\n1 <= month <= 12')
            ok = 0
        if not (1 <= int(matchObj.group(3)) <= 31):
            messagebox.showinfo(
                'Wrong value', 'Wrong number of day.\n1 <= day <= 31')
            ok = 0
        if not (1 <= int(matchObj.group(4)) <= 7):
            messagebox.showinfo(
                'Wrong value', 'Wrong number of week_day.\n1 <= week_day <= 7')
            ok = 0
        if not (0 <= int(matchObj.group(5)) <= 23):
            messagebox.showinfo(
                'Wrong value', 'Wrong number of hours.\n0 <= hours <= 23')
            ok = 0
        if not (0 <= int(matchObj.group(6)) <= 59):
            messagebox.showinfo(
                'Wrong value', 'Wrong number of minutes.\n0 <= minutes <= 59')
            ok = 0
        if not (0 <= int(matchObj.group(7)) <= 59):
            messagebox.showinfo(
                'Wrong value', 'Wrong number of seconds.\n0 <= seconds <= 59')
            ok = 0

        if (ok == 1):  # if data ok
            time_date = time_date + "\0"  # '\0' for C-string
            serialPort.write(time_date.encode('Ascii'))
    else:
        messagebox.showinfo(
            'Wrong value', 'Please type two-digit number for every segment.\nyear:month:day:week_day:hours:minutes:seconds\nFor example 20:12:31:1:12:30:30')


# -----------------------------------------------------------------------
# The function is read the seconds and average from the label,
# check the errors and send it to the TTL port
def change_time_upload():
    time_upload = entry_ch_up.get()

    matchObj = re.match(  # regular expression, positive number for every part
        r"^(\d+):(\d+)$", time_upload)
    ok = 1

    if matchObj:
        if not (1 <= int(matchObj.group(1)) <= 1200):
            ok = 0
            messagebox.showinfo(
                'Wrong value', 'Must be a positive integer.\n1 <= seconds <= 1200')
        if not (3 <= int(matchObj.group(2)) <= 1000):
            ok = 0
            messagebox.showinfo(
                'Wrong value', 'Must be a positive integer.\n1 <= average <= 1000')

        if (ok == 1):  # if data ok
            # 'a' for designation the time and average upload, '\0' for C-string
            time_upload = "a" + time_upload + "\0"
            serialPort.write(time_upload.encode('Ascii'))

    else:
        messagebox.showinfo(
            'Wrong value', 'Please type positive number for every segment.\nseconds:average\nFor example 3:10')


# -----------------------------------------------------------------------
# The function is read the data from the TTL port,
# check the errors and update the labels
def last_update():
    try:
        global flag_ch_port
        global flag_com
        global serialPort
        global flag_delete_thread
        serialString = ""

        serialPort.flushInput()
        serialPort.flushOutput()  # flush the TTL port

        while (1):
            if (flag_delete_thread == 1):
                serialPort.close()
                break

            if (flag_ch_port == 1):
                flag_ch_port = 0
                serialPort.close()

                if (flag_com == 1):
                    flag_com = 0
                    lbl_ch_com.configure(text="{0}".format("UART"))

                    while(1):
                        try:
                            serialPort = serial.Serial(port=UART_COM, baudrate=115200,
                                                       bytesize=8, timeout=0.5, stopbits=serial.STOPBITS_ONE)
                            # 'u' for main port, '\0' for C-string
                            serialPort.write("u\0".encode('Ascii'))
                            break

                        except serial.serialutil.SerialException:
                            print("Could not open port {0}".format(UART_COM))
                            time.sleep(2)
                else:
                    flag_com = 1
                    lbl_ch_com.configure(text="{0}".format("NRF24"))

                    while(1):
                        try:
                            serialPort = serial.Serial(port=NRF24_COM, baudrate=115200,
                                                       bytesize=8, timeout=0.5, stopbits=serial.STOPBITS_ONE)
                            # 'n' for slave port, '\0' for C-string
                            serialPort.write("n\0".encode('Ascii'))
                            break

                        except serial.serialutil.SerialException:
                            print("Could not open port {0}".format(NRF24_COM))
                            time.sleep(2)

            if not q_receive.empty():
                item = q_receive.get()
                item = item + b'\0'
                serialPort.write(item)

            if(serialPort.in_waiting > 0):
                serialString = serialPort.readline()
                serialString = serialString.decode('Ascii')

                matchObj = re.match(  # regular expression, three floating point number and two digits numbers for every part
                    r"^;([+-]?([0-9]*[.])?[0-9]+):([+-]?([0-9]*[.])?[0-9]+):([+-]?([0-9]*[.])?[0-9]+):(\d+):(\d+):(\d+):(\d+):(\d+):(\d+):(\d+):(\d+):(\d+);$", serialString)

                if matchObj:

                    if (flag_queue == 1):
                        q_send.put(matchObj.group(0))

                    lbl_date.configure(
                        text="Year: {0}    Month: {1}    Day: {2}    WeekDay: {3}".format(matchObj.group(7),
                                                                                          matchObj.group(8), matchObj.group(9), matchObj.group(10)))
                    lbl_time.configure(
                        text="Hours: {0}    Minutes: {1}    Seconds: {2}".format(matchObj.group(11),
                                                                                 matchObj.group(12), matchObj.group(13)))

                    lbl_humi.configure(
                        text="{0} %".format(matchObj.group(1)))
                    lbl_tem.configure(text="{0} C".format(matchObj.group(3)))
                    lbl_adc.configure(text="{0} V".format(matchObj.group(5)))
                    lbl_probe.configure(
                        text="{0} sec".format(matchObj.group(14)))
                    lbl_average.configure(text="{0} units".format(
                        matchObj.group(15)))  # update labels
                else:
                    print("not mach")
                    print(serialString)  # print the data that not match

    except Exception as e:
        print(e)


# -----------------------------------------------------------------------
# The function change the flag, which defines the ports
def change_com():
    global flag_ch_port

    flag_ch_port = 1


# -----------------------------------------------------------------------
# The function delete the window and thread after 0.5 seconds
def on_delete_window():
    global flag_delete_thread

    flag_delete_thread = 1
    window.after(700, window.destroy)


def onServer():
    global flag_queue
    global flag_delete_thread
    setdefaulttimeout(0.5)

    try:
        s = socket()
        s.bind(('', 12345))
        s.listen(5)
    except Exception as e:
        print(e)
        sys.exit(0)

    print("Waiting on {}:{}".format(*s.getsockname()))

    while True:
        try:
            client, address = s.accept()
            flag_queue = 1
            break
        except timeout:
            if (flag_delete_thread == 1):
                sys.exit(0)
            else:
                continue

    print("ip: {}   port: {} connected".format(*address))

    while True:
        try:
            if not q_send.empty():
                item = q_send.get()
                client.send(item.encode())
                q_send.task_done()

            msg = client.recv(1000)

            if msg == b'':
                flag_queue = 0
                client.close()

                while True:
                    try:
                        client, address = s.accept()
                        flag_queue = 1
                        break
                    except timeout:
                        if (flag_delete_thread == 1):
                            sys.exit(0)
                        else:
                            continue

            else:
                q_receive.put(msg)
                q_receive.task_done()

        except timeout:
            if (flag_delete_thread == 1):
                flag_queue = 0
                sys.exit(0)
            else:
                continue
        except (BlockingIOError, ConnectionResetError) as e:
            print(e)
            flag_queue = 0
            sys.exit(0)

    #
    # ===================================================================================
    #
    # ====================================================
NRF24_COM = "COM3"
UART_COM = "COM8"
flag_com = 1
flag_ch_port = 0
flag_delete_thread = 0
flag_queue = 0

q_send = queue.Queue()
q_receive = queue.Queue()
# ====================================================


while (1):
    try:
        serialPort = serial.Serial(port=NRF24_COM, baudrate=115200,
                                   bytesize=8, timeout=0.5, stopbits=serial.STOPBITS_ONE)
        break
    except serial.serialutil.SerialException:
        print("Could not open port {0}".format(NRF24_COM))
        time.sleep(2)

while (1):
    try:
        thread_last_up = Thread(target=last_update)
        thread_last_up.start()

        thread_tcp = Thread(target=onServer)
        thread_tcp.start()
        break

    except Exception as e:
        print("Thread error: ", e)


window = Tk()
window.title("Weather Analyzer")
window.geometry("490x250")

lbl_last = Label(window, text="Last update")
lbl_last.grid(column=1, row=0)

lbl_date = Label(window, text="Year: 10    Month: 5    Day: 1    WeekDay: 1")
lbl_date.grid(column=1, row=1)

lbl_time = Label(window, text="Hours: 12    Minutes: 12    Seconds: 30")
lbl_time.grid(column=1, row=2)


lbl_probe_txt = Label(window, text="Probe time: ")
lbl_probe_txt.grid(column=0, row=3)

lbl_probe = Label(window, text="3 sec")
lbl_probe.grid(column=1, row=3)


lbl_average_txt = Label(window, text="Average: ")
lbl_average_txt.grid(column=0, row=4)

lbl_average = Label(window, text="10 units")
lbl_average.grid(column=1, row=4)


lbl_humi_txt = Label(window, text="Humidity: ")
lbl_humi_txt.grid(column=0, row=5)

lbl_humi = Label(window, text="10.99999 %")
lbl_humi.grid(column=1, row=5)


lbl_tem_txt = Label(window, text="Temperature: ")
lbl_tem_txt.grid(column=0, row=6)

lbl_tem = Label(window, text="20.99999 C")
lbl_tem.grid(column=1, row=6)

lbl_adc_txt = Label(window, text="ADC Voltage: ")
lbl_adc_txt.grid(column=0, row=7)

lbl_adc = Label(window, text="1.99999 V")
lbl_adc.grid(column=1, row=7)


lbl_ch_time_data = Label(window, text="Change time and date")
lbl_ch_time_data.grid(column=0, row=8)

entry_ch_time_data = Entry(window, width=40)
entry_ch_time_data.grid(column=1, row=8)

btn_ch_time_data = Button(window, text="OK", command=change_time_date)
btn_ch_time_data.grid(column=2, row=8)

lbl_ch_up = Label(window, text="Change time and average")
lbl_ch_up.grid(column=0, row=9)

entry_ch_up = Entry(window, width=40)
entry_ch_up.grid(column=1, row=9)

btn_ch_up = Button(window, text="OK", command=change_time_upload)
btn_ch_up.grid(column=2, row=9)


lbl_ch_com = Label(window, text="NRF24")
lbl_ch_com.grid(column=1, row=10)

btn_ch_com = Button(window, text="OK", command=change_com)
btn_ch_com.grid(column=2, row=10)

# change function of delete
window.protocol('WM_DELETE_WINDOW', on_delete_window)

window.mainloop()
