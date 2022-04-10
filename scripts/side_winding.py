try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import math


def acq_handle(count):
    servo = [0]*count
    for i in range(count):
        name = "servo_"+str(i)
        res, servo[i] = sim.simxGetObjectHandle(clientID, "Servo_joint_"+str(i), sim.simx_opmode_blocking)
        if res == sim.simx_return_ok:
            print("Servo_"+str(i)+" handle acquired successfully")
        else:
            print('Remote API function call returned with error code: ', res)
    return servo


if __name__ == '__main__':

    print('Program started')
    sim.simxFinish(-1)  # just in case, close all opened connections
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # Connect to CoppeliaSim
    if clientID != -1:
        print('Connected to remote API server')
        servo_cnt = 8
        counter = 0
        frequency = 0.3
        amplitude = 1
        # offset = 25
        delayTime = 50/1000
        lag = math.pi/4

        # Now try to retrieve data in a blocking fashion (i.e. a service call):
        servo = acq_handle(servo_cnt)

        time.sleep(2)
        servo_opmode = sim.simx_opmode_blocking

        while(True):

            '''for i in range(servo_cnt):
                if ((i % 2) == 0):
                    sim.simxSetJointTargetPosition(clientID, servo[i], (amplitude * math.cos(i * lag)), sim.simx_opmode_blocking)
                else:
                    sim.simxSetJointTargetPosition(clientID, servo[i], (amplitude * math.cos(i * lag)), sim.simx_opmode_blocking)'''

            time.sleep(2)
            for counter in range(360):
                time.sleep(delayTime)
                # for short program uncomment below section
                for i in range(servo_cnt):
                    if ((i % 2) == 0):
                        sim.simxSetJointTargetPosition(clientID, servo[i], (amplitude * math.cos(2 * frequency * counter * math.pi + i * lag + math.pi/4)), servo_opmode)
                    else:
                        sim.simxSetJointTargetPosition(clientID, servo[i], (amplitude * math.cos(2 * frequency * counter * math.pi + i * lag)), servo_opmode)

                '''sim.simxSetJointTargetPosition(clientID, servo[0], (amplitude * math.cos(2 * frequency * counter * math.pi + 0 * lag + math.pi/4)), servo_opmode)
                time.sleep(100/1000)
                sim.simxSetJointTargetPosition(clientID, servo[2], (amplitude * math.cos(2 * frequency * counter * math.pi + 2 * lag + math.pi/4)), servo_opmode)
                time.sleep(100/1000)
                sim.simxSetJointTargetPosition(clientID, servo[4], (amplitude * math.cos(2 * frequency * counter * math.pi + 4 * lag + math.pi/4)), servo_opmode)
                time.sleep(100/1000)
                sim.simxSetJointTargetPosition(clientID, servo[6], (amplitude * math.cos(2 * frequency * counter * math.pi + 6 * lag + math.pi/4)), servo_opmode)
                time.sleep(100/1000)

                sim.simxSetJointTargetPosition(clientID, servo[1], (amplitude * math.cos(2 * frequency * counter * math.pi + 1 * lag)), servo_opmode)
                time.sleep(100/1000)
                sim.simxSetJointTargetPosition(clientID, servo[3], (amplitude * math.cos(2 * frequency * counter * math.pi + 3 * lag)), servo_opmode)
                time.sleep(100/1000)
                sim.simxSetJointTargetPosition(clientID, servo[5], (amplitude * math.cos(2 * frequency * counter * math.pi + 5 * lag)), servo_opmode)
                time.sleep(100/1000)
                sim.simxSetJointTargetPosition(clientID, servo[7], (amplitude * math.cos(2 * frequency * counter * math.pi + 7 * lag)), servo_opmode)
                time.sleep(100/1000)'''

        # sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
        # while time.time()-startTime < 5:
        #     returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
        #     if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        #         print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
        #     time.sleep(0.005)

        # Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(
            clientID, 'Hello CoppeliaSim!', sim.simx_opmode_oneshot)

        # # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        # sim.simxGetPingTime(clientID)

        # # Now close the connection to CoppeliaSim:
        # sim.simxFinish(clientID)
    else:
        print('Failed connecting to remote API server')
    print('Program ended')
