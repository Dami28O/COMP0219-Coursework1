# COMP0219-Coursework1
Coursework1 for COMP0219 - Sensors, Sensing and Signal Conditioning

(WILL MAKE THIS A BIT TIDIER AT SOME POINT)

## STM CUBE FILES
[main.c](stmcube_cw1/Core/Src/main.c) outputs at 200Hz from the load cell, and at the current moment dummy values from the variable "cup."
- Scale offset needs to be properly calibrated (ideally in the wind tunnel on wednesday?)


## DATA LOGGING 

Data found in the [data](data) directory. This directory contains raw csv files generated from the [data logging script](datalog.py).

#### How to use it?
```
"Usage: python3 datalog.py <output_file> [<port>] [<baudrate>]"
```

- Current default port set to my mac port and may be different on windows but the baudrate should remain the same. Both port and baudrate are optional command line arguments but if the port changes I would recommend using it as an argument. 
- Secondly, you cannot run this script and also look at readings in "RealTerm" or "CoolTerm". So to log data, disconnect from the other terminal, run [datalog.py](datalog.py) and reset the board. It should output the preliminary text regarding calibration but will not log them to csv, so you will have some idea of what is going on. 
    -   *Note it will appear like nothing is happening but trust the process, you will hopefully see an updated csv one you press "CTRL+C" to interrupt the data logging

**CSV**
The CSV contains three columns:
1. The first column represents a timestamp, taken from the computer's internal clock as opposed to the STM's clock
2. Load Cell Raw Data, currently in grams but could be scaled to output the Force and to some extent the approximated windspeed
3. This column outputs the ticks from the encoder

- It is also possible to add more states and statistics but make sure the order and amount matches that being sent over UART from the STM device, otherwise the datalog.py will reject the output :)

## DATA VISUALISATION

#### Prerequisites
```
pip install requirements.txt
```

- Additional dependencies such as torch, keras or tensorflow could be added as well depending on further implementation.

#### USE
Navigate to the [regression.ipynb](regression.ipynb) and install the prerequisites. It would be wise to use a virtual environment.

- Code is pretty straightforward. Replace "filename" with the file you want to plot and from there you should be able to see how the visualisation over time