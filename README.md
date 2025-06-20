# PROJECT GOAL

Create a system to acquire a signal through **ADC** (Analog to Digital Converter)\
The **sampling frequency** must be **1kHz**.\
When the number of acquired samples reach **500**, the data must be copied from one buffer to another.\
The entire system must work in **FreeRTOS**.\
Every 500 samples we want to **estimate the signal frequency** using the **zero crossing method**.\
(suppose the signal is **sinusoidal** and it's generated from the outside)\
The result must be send to the **asynchronous serial port**.

# INPUT

In STM32CubeMx the **ADC1: IN13 Single-ended** was activated\
The ADC input pin is **PA6**\
**The voltage range is 0V to 3.3V**

# OUTPUT

The estimated signal frequency (expressed in Hz) is provided using the **UART3 serial port**

# PROGRAM STRUCTURE

This program is composed of:

- **3 RTOS Tasks** (Idle Task, Frequency Estimation Task and Serial Print Task)
- **2 buffers** (one used for sampling, one used for frequency estimation)
- **1 RTOS periodic Timer** (used to trigger a signal sampling)
- **1 RTOS binary Semaphore** (used to signal that we reach the number of samples needed for the frequency estimation)
- **1 RTOS Queue** (used to communicate the estimated frequency to the Serial Print task)
- **1 Timer Callback** (handles the ADC conversion and writes the result to the buffer)
- **1 Utility Function** (used to copy the content of the ADC buffer to the processing buffer)
