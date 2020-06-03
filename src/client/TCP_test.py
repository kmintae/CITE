from socket import *
# from C_RobotArm.RobotArmClient import armClient
import yaml
import time

# from C_Motor.Car import Car

# Configurations
config = yaml.load(open("config/client.yaml", 'r'), Loader=yaml.FullLoader)

MAX_BUF_SIZE = 1024

def run_client():
    # numInst: Calculating # of Instructions
    numInst = 0

    clientSock = socket(AF_INET, SOCK_STREAM)
    clientSock.connect((config["SERVER_IP_ADDR"], config["SERVER_PORT"]))

    # New Car Module
    # car = Car()

    # robotArm = armClient(config["GPIO_ARM_DIRPINS"], config["GPIO_ARM_STPPINS"], config["ROBOTARM_MIN_ANGLES"],
    #                     config["ROBOTARM_MAX_ANGLES"], config["GPIO_SERVO_PIN"])

    # Client Flow 1 : Send Robot_Arm Number & Color Data
    sendInfo = str(config["ROBOT_ARM_NUM"]) + " " + str(config["ROBOT_ARM_COLOR"][0]) + " "
    sendInfo += str(config["ROBOT_ARM_COLOR"][1]) + " " + str(config["ROBOT_ARM_COLOR"][2]) + " "
    sendInfo += str(config["INIT_POS"][0]) + " " + str(config["INIT_POS"][1])

    sendInfo = sendInfo.encode()
    clientSock.sendall(sendInfo)

    # Client Flow 2 : Iteration with While Loop, Executing action for robot arm instructions
    while (True):
        recvInst = (clientSock.recv(MAX_BUF_SIZE).decode()).split(' ')

        if (recvInst[0] == 'RRT'):   # Right Rotation
            print("#" + numInst + ": RRT " + recvInst[1])
            # TODO: car.rotate(float(recv_inst_tok[1]))
        elif (recvInst[0] == 'FWD'): # Forward
            print("#" + numInst + ": FWD " + recvInst[1])
            # TODO: car.move_forward(float(recv_inst_tok[1]))
        elif (recvInst[0] == 'ARM'): # Arm Movement
            print("#" + numInst + ": ARM " + recvInst[1] + " " + recvInst[2] + " " + recvInst[3] + " " + recvInst[4])
            # TODO: ARM arg[1] arg[2] arg[3] state
            state = int(recvInst[4])

            # TODO : Implement
            if (state == 0):  # GRAB
                # robotArm.work([recv_inst_tok[1], recv_inst_tok[2], recv_inst_tok[3]], True)
                pass
            else:  # Release
                pass

        elif (recvInst[0] == 'HLT'): # Halt
            print("#" + numInst + ": HLT")

        elif (recvInst[0] == 'DCN'): # Disconnect
            print("#" + numInst + ": DCN")
            break

        # TODO: Remove (Test Purpose Only)
        time.sleep(2)

        # Noticing Current Task is totally done.
        clientSock.sendall("DONE".encode())
        numInst = numInst + 1

    clientSock.close()