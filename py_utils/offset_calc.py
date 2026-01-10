from statistics import mean

motor1 = [[5.5370, 5.5655], [5.6766, 5.6853], [5.7538, 5.7688], [5.6467, 5.6595], [5.6139, 5.6440], [5.7405, 5.7561], [5.6635, 5.7000]]
motor2 = [[3.9079, 3.9427], [4.0755, 4.1122], [4.0478, 4.0776], [3.9724, 3.9755], [4.0143, 4.0206], [4.0766, 4.0920], [4.0079, 4.0100]]

motors = [motor1, motor2]

for motor in motors:
    mids = []
    for nums in motor:
        mid = (nums[0] + nums[1]) / 2
        mids.append(mid)
    print(round(mean(mids), 4))