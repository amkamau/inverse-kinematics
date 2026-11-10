import xarm

arm = xarm.Controller('USB', debug=True)

for i in range(1, 7):
    position = arm.getPosition(i, degrees=False)
    print(f'Servo {i}: {position}')

print('Voltage:', arm.getBatteryVoltage())
