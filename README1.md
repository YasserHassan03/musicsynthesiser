# System Task Identification and Implementation Methods
In this section the tasks and interrupts are described along with any measures taken to ensure that shared resources are accessed safely.

## sampleISR
In this interrupt, the waveforms for the pressed keys are generated using a phase accumulator. The current step sizes are used in a phase accumulator array in order to generate a waveform of each pressed keys' frequency. The array is then summed and scaled in order to generate an output voltage between 0 and 255.  The current step size array is a shared resource and is therefore protected with 


## Advanced features
### Polyphony
### Waveform generation
Sawtooth, square, triangle and sine waves were implemented with the three former being calculated algorithmically. The sine waves were implemented using a look-up table which was filled during setup with 255 scaled (-127 to 127) single-precision floating point samples of a sine cycle. A lookup table was used because calculating the sine function was found to be computationally intense and when using a maclaurin series to approximate the function the quality of the sine wave deteriorated significantly. 
### Pitch bending
### User Interface
