# Initiation Interval, Execution Time, and Critical Instant Analysis

In this discussion, we delve into the concepts of Critical Instant Analysis (CIA), CPU utilization, and task execution time.

# Python CIA Script
To perform CIA analysis, we've developed a Python script named `cia.py`. This script organizes tasks into Task objects, each comprising the task's execution time and its period or initiation interval. This setup proves invaluable, especially when determining worst-case execution times for individual tasks.

Upon supplying properly structured Task objects to the critical_instant_analysis() function, the script arranges them by priority. Priority is inversely proportional to the period (larger period implies higher priority). Subsequently, the script identifies the task with the longest period and stores it as response_time. Then, it calculates the total execution time for each task within one period of the task with the lowest priority. This calculation ensures that we consider the ceiling value when determining the number of task iterations. Finally, a comparison between the total execution time and the period of the lowest priority task determines whether the tasks are schedulable, and this outcome is conveyed through a print statement.




# Computing CPU Utilization
Within the same `cia.py` script, we compute CPU utilization by iterating through all tasks, dividing their execution time by their period, and summing these values. This sum is then converted into a percentage and printed for easy understanding. 



# Minimum Theoretical Bound on Execution time 


The minimum bound on the initiation interval can be calculated by setting the highest priority to the task with the lowest execution time. It is also assumed that the overhead to context switching is 0. Under these assumptions, we checked that the initiation interval of the current task is met under worst case conditions. The worst case being that all other tasks with higher priority run during the time which the current task is running. The results show that the minimum initiaion interval for the following tasks are.

| Task          | Theoretical Minimum |
|---------------|---------------------|
| Sample ISR    |                     |
| Transmit ISR  |                     |
| Recieve ISR   |                     |
| Transmit Task |                     |
| ScanKeys task |                     |
| Mixer Task    |                     |
| Decode Task   |                     |
| Display Task  |                     |



However these values are not possible to achieve because there is an overhead to context switching. The FreeRTOS Config is set to trigger an interrupt ever 1ms which the scheduler uses to switch between threads. This means that there is a minimum switching overhead of 1ms between tasks. Hence the theoretical values are not attainable.


# Setting Initiaion intervals 

In order to attain reasonable initation intervals which met the specification. We did trial and error ensuring that the code ran and there was no perceptible delay when pressing keys as well as ensuring messages and advanced functionaly behaved as expected. We found those values to be

| Task          | Theoretical Minimum |
|---------------|---------------------|
| Transmit Task |  set by scheduler   |
| ScanKeys task |     50ms            |
| Mixer Task    |      100ms               |
| Decode Task   |      150ms               |
| Display Task  |    100ms(according to specs)                 |

The Transmit Task was set by the scheduler as its execution time was found to be minimal.