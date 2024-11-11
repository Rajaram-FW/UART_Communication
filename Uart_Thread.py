import threading #threads used to run a non-blocking code 
import serial
import time

# Create a Serial object
ser = serial.Serial('COM6', baudrate=2400, timeout=7)  # Adjust baud rate if needed
a = 1
Send_Loop_Value = 1 #Send message only once
# Function to send data through UART
def send_data():
    global a
    while True:
        if(a==Send_Loop_Value):
            data = "Finance Minister Arun Jaitley Tuesday hit out at former RBI governor Raghuram Rajan for predicting that the next banking crisis would be triggered by MSME lending, saying postmortem is easier than taking action when it was required. Rajan, who had as the chief economist at IMF warned of impending financial crisis of 2008, in a note to a parliamentary committee warned against ambitious credit targets and loan waivers, saying that they could be the sources of next banking crisis. Government should focus on sources of the next crisis, not just the last one. In particular, government should refrain from setting ambitious credit targets or waiving loans. Credit targets are sometimes achieved by abandoning appropriate due diligence, creating the environment for future NPAs,\" Rajan said in the note.\" Both MUDRA loans as well as the Kisan Credit Card, while popular, have to be examined more closely for potential credit risk. Rajan, who was RBI governor for three years till September 2016, is currently"  # Data to send
            ser.write(data.encode())  # Write data to UART
            # print(f"Sent: {data}") #uncomment if you want to print Tx data
            a+=1
            time.sleep(1)  # Wait for 1 second before sending again

# Function to receive data from UART
def receive_data():
    while True:
        if ser.in_waiting > 0:  # Check if there's any data available to read
            data = ser.readline()  # Read a full line (ending with newline \n)
            # data = ser.read(ser.in_waiting)
            dataInstring = str(data,'UTF-8')
            print(dataInstring)
            print("\n-----------sucessfully received and printed-------------\n")
        else:
            time.sleep(0.1)  # Sleep briefly to prevent high CPU usage

# Create and start send and receive threads
send_thread = threading.Thread(target=send_data, daemon=True)  # daemon=True allows the thread to exit when the program ends
receive_thread = threading.Thread(target=receive_data, daemon=True)

#start both the threads which will make them run parallely bciz daemon is true for both threads
send_thread.start()  # Start sending data
receive_thread.start()  # Start receiving data

# Keep the main thread alive
try:
    while True:
        time.sleep(1)  # Main thread sleeps, allowing send/receive threads to run parallely
except KeyboardInterrupt:
    print("\nProgram interrupted by user")
finally:
    ser.close()  # Close the serial connection when done
