from socket import *
# from C_RobotArm.RobotArmClient import armClient

# from C_Motor.Car import Car

# Configurations
MAX_BUF_SIZE = 1024

def run_client():
    # numInst: Calculating # of Instructions
    numInst = 0

    clientSock = socket(AF_INET, SOCK_DGRAM)
    clientSock.connect(('141.223.198.221', 61557))

    clientSock.sendall("Request".encode())

    while (True):
        recvInst = clientSock.recv(MAX_BUF_SIZE).decode()
        print(recvInst)

    clientSock.close()