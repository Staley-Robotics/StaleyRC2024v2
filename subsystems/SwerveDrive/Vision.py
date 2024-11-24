from ntcore import NetworkTableInstance, NetworkTableEntry, IntegerSubscriber

from wpilib import DriverStation as DV
from wpimath.units import degreesToRadians
from wpimath.geometry import Pose2d

from commands2 import Subsystem

class Camera:
    def __init__(self, nt_name:str):
        table = NetworkTableInstance.getDefault().getTable(nt_name)
        
        self.tv: IntegerSubscriber = table.getIntegerTopic('tv').subscribe(0)
        self.botpose = table.getDoubleArrayTopic('botpose_wpiblue').subscribe([])
    
    def convertToPose2d(self, poseData:list[float]) -> Pose2d:
        return Pose2d(poseData[0], poseData[1], degreesToRadians(poseData[5]))
    
    def getLastUpdate(self) -> tuple[Pose2d, float] | None:
        '''
        if the camera has data:
        returns a tuple with the measured Pose2d and latency
        else: 
        returns None
        '''
        has_data = self.tv.get()
        if has_data:
            data = self.botpose.get()
            return (self.convertToPose2d(data), data[6])
        return None

class Vision:

    NUM_CAMERAS = 4

    def __init__(self):
        self.cameras = [
            Camera('limelight-one'),
            Camera('limelight-two'),
            Camera('limelight-three'),
            Camera('limelight-four')
        ]
    
    def getVisionData(self) -> list[tuple[Pose2d, float]]:
        '''
        returns an array of tuples containing a measured Pose2d and the associated latency
        '''
        output = []
        for camera in self.cameras:
            data = camera.getLastUpdate()
            if data:
                output.append(data)
        

        
