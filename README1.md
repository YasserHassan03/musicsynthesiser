# System Task Identification and Implementation Methods
In this section the tasks and interrupts are described along with any measures taken to ensure that shared resources are accessed safely.

## Sample ISR
In this interrupt, the waveforms for the pressed keys are generated using a phase accumulator. The current step sizes are used in a phase accumulator array in order to generate a waveform of each pressed keys' frequency. The array is then summed and scaled in order to generate an output voltage between 0 and 255.  The current step size array is a shared resource and is therefore accessed using atomic operations; only the sampleISR reads data from this resource so an atomic load is sufficient. Another shared resource accessed in this function is a global variable which stores the position of the joystick in the y-axis. Similarly, this is the only process that reads the variable so an atomic load operation is sufficient to ensure thread safety.

## CAN TX/RX ISR
CAN TX ISR releases the MUTEX set by the transmit task once the transaction has been successfully completed. The CAN RX ISR gets the message data and xQueueSendFromISR() places the data in the queue. 

## Scan keys task
The scan keys task scans the key matrix and updates all members of the context. It also sends a CAN message depending on state changes (if a new receiver or waveform is chosen).

## Decode task
The decode task decodes incoming messages over the can interface and updates the relevant members of the context. For example if a new waveform is set on one of the keyboards a message is sent to notify all keyboards on the bus. This is decoded to set the waveforms on each keyboard so they all match.

## Transmit task
The transmit task transmits the messages on the queue.

## Mixing task
The mixing task updates the step size array depending on the incoming notes and octaves.


## Advanced features
### Polyphony
Polyphony was implemented by creating an array of phase accumulators one value for each note. This was then summed and scaled in order to generate a value for the voltage to send to the speaker.
### Waveform generation
Sawtooth, square, triangle and sine waves were implemented with the three former being calculated algorithmically. The sine waves were implemented using a look-up table which was filled during setup with 255 scaled (-127 to 127) single-precision floating point samples of a sine cycle. A lookup table was used because calculating the sine function was found to be computationally intense and when using a maclaurin series to approximate the function the quality of the sine wave deteriorated significantly. 
### Pitch bending
Pitch bending can be achieved by moving the joystick in the y direction. A movement in the positive y-direction results in an increase in pitch and in the negative-y a decrease. This is achieved by reading from the shared yAxis resource using an atomic load and then scaling it in order to raise or decrease the frequency by around a semitone in the extreme positions.
### User Interface
