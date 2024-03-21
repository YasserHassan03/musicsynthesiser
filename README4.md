# Inter-Task Blocking Dependencies and Deadlock Analysis
Queue send is blocking when the queue is full. Queue receive is blocking when the queue is empty. This is due to port_MaxDelay, which is indefinite time running, which is approximately 49 days. This block would crash the system.
CAN Tx is a blocking operation as it waits for the CAN controller to be free. To ensure that it doesn't block indefinitely the CAN controller is only sent to if other nodes are recognized on the bus using the west and east pin. 

