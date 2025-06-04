# Here you define the commands that will be added to your add-in.

# Import the inertia calculator module
from .inertiaCalculator import entry as inertiaCalculator

# List of commands - only the inertia calculator
commands = [
    inertiaCalculator
]


# Assumes you defined a "start" function in each of your modules.
# The start function will be run when the add-in is started.
def start():
    for command in commands:
        command.start()


# Assumes you defined a "stop" function in each of your modules.
# The stop function will be run when the add-in is stopped.
def stop():
    for command in commands:
        command.stop()