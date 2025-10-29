import serial
import csv
from datetime import datetime
import time
import os
import sys


def main():

    # --- Default Config ---
    def_port = '/dev/tty.usbmodem11103'           # or '/dev/ttyACM0' on Linux/Mac
    def_baudrate = 115200

    # Get user input from command line (Port and baudrate are optional command line arguments)
    if len(sys.argv) < 2:
        print("Usage: python3 datalog.py <output_file> [<port>] [<baudrate>]")
        sys.exit(1)

    # Extract user inputs    
    output_file = sys.argv[1]
    print(f"Output file: {output_file}")
    
    # Save data to a 'data' folder
    if not os.path.exists('data'):
        os.makedirs('data')
    output_file = os.path.join('data', output_file)

    # If optional arguments are provide, use them; otherwise, use defaults
    if len(sys.argv) >= 3:
        port = sys.argv[2]

        if len(sys.argv) == 4:
            baudrate = int(sys.argv[3])
        else:
            baudrate = def_baudrate  # default baudrate
    else:
        port = def_port  # default port
        baudrate = def_baudrate  # default baudrate
    

    # --- Open Serial ---
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # allow connection to settle


    # --- Prepare CSV ---
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "load_cell_raw", "cup_ticks"])  # adjust columns

        # --- Read and Log Data ---
        print("Logging... press Ctrl+C to stop.")
        try:
            while True:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    continue
                
                # Example line from MCU: "1234,56,2.45"
                try:
                    values = [float(x) for x in line.split(',')]
                    writer.writerow([time.time()] + values)
                    # csv.flush()
                except ValueError:
                    print("Skipped line:", line)
        except KeyboardInterrupt:
            print("\nLogging stopped.")

    ser.close()

if __name__ == "__main__":
    main()