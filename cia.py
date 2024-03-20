import math

class Task:
    def __init__(self, execution_time, period, deadline=None):
        self.execution_time = execution_time
        self.period = period
        # self.deadline = deadline

def critical_instant_analysis(tasks):
    tasks.sort(key=lambda x: x.period)  

    cpu_utilization = sum([t.execution_time / t.period for t in tasks])

    response_time = tasks[-1].period
    workload = sum([math.ceil(response_time / t.period) * t.execution_time for t in tasks]) 

    if workload > response_time:
        print("Fail")
    else:
        print("All tasks are schedulable, Pass")

    print(f"CPU Utilization: {round(cpu_utilization * 100, 2)}%")

tasks = [Task(0.1, 6), Task(1,33), Task(750, 1000)]
critical_instant_analysis(tasks)


