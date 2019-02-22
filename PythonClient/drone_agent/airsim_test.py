import setup_path 
import airsim
import time

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

client.armDisarm(True)

landed = client.getMultirotorState().landed_state
if landed == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    print("already flying...")
    client.hoverAsync().join()

print("move")
client.moveByVelocityAsync(1, 1, -1, 1).join()
time.sleep(1)
print("move2")
client.moveByVelocityAsync(-1, -1, -1, 1).join()