import socket
import sys

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 39002))

while True:
    data, addr = sock.recvfrom(2048)
    nums = []
    count = 0
    for i in range(0, len(data), 4):
        count = count + 1
        if count > 3 and count % 3 != 0:
            nums.append(int.from_bytes(data[i:i+3], 'little', signed=True))
        else:
            nums.append(int.from_bytes(data[i:i+3], 'little', signed=True))
    print("Number of objects: " + str(nums[2]))
    for i in range(3, len(nums), 3):
        print("Status = " + str(nums[i]))
        print("D = " + str(nums[i+1]) + "mm")
        print("S = " + str(nums[i+2]) + "mm")
