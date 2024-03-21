# Shared Data Structures and Synchronisation Methods

## Context
A context class is used to store and update values of volume, waveform, octave and sender/reciever. There are also member functions to lock and unlock the mutex.

## stepArray
The step array contains the stepsizes for keys that are pressed and zero otherwise. This is only read from in the sampleISR and used to generate waveforms using a phase accumulator. It is only written to from the mixing task. As such, atomic load operations are used for loading data from this shared resource into the sampleISR process and atomic store operations are used in the mixingTask.

## yAxis
This variable is used to store the value of the joystick in the y-direction. Similarly, it is only written to from the display task and read from in the sampleISR task so atomic load and store operations are used. 

