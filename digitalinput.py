import time
import board
import digitalio

button = digitalio.DigitalInOut(board.G0)
button.direction = digitalio.Direction.INPUT

count = 0
while True:
    print(f"[Iteration {count}] Button value: {button.value}")
    time.sleep(5)
    count += 2