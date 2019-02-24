import setup_path 
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True, vehicle_name='Drone1')

client.armDisarm(True, vehicle_name='Drone1')

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync(vehicle_name='Drone1').join()
else:
    print("already flying...")
    client.hoverAsync(vehicle_name='Drone1').join()

print("move")
client.moveByVelocityAsync(1, 1, 0, 2, vehicle_name='Drone1').join()
print(client.getMultirotorState(vehicle_name='Drone2').kinematics_estimated.position)
client.moveByVelocityAsync(0, 1, 0, 2, vehicle_name='Drone1').join()
print(client.getMultirotorState(vehicle_name='Drone2').kinematics_estimated.position)
time.sleep(1)
print("move2")
client.moveByVelocityAsync(-1, -1, 0, 2, vehicle_name='Drone1').join()
print(client.getMultirotorState(vehicle_name='Drone2').kinematics_estimated.position)