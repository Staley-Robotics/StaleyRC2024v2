from wpimath import applyDeadband
from commands2.button import CommandXboxController

class FalconXboxController(CommandXboxController):
    def __init__(self, port:int, deadband:float = 0.04):
        super().__init__(port)
        self.__deadband = deadband

    def getLeftUpDown(self) -> float:
        """
        Get the Up/Down axis value of left side of the controller.
        
        Additional Features:
        - Integrates Deadband
        - Positive Forward

        :returns: The axis value.
        """
        return applyDeadband( -super().getLeftY(), self.__deadband )
    
    def getLeftSideToSide(self) -> float:
        """
        Get the Side to Side axis value of left side of the controller.
        
        Additional Features:
        - Integrates Deadband
        - Positive Left

        :returns: The axis value.
        """
        return applyDeadband( -super().getLeftX(), self.__deadband )
    
    def getRightUpDown(self) -> float:
        """
        Get the Up/Down axis value of right side of the controller.
        
        Additional Features:
        - Integrates Deadband
        - Positive Forward

        :returns: The axis value.
        """
        return applyDeadband( -super().getRightY(), self.__deadband )
    
    def getRightSideToSide(self) -> float:
        """
        Get the Side to Side axis value of right side of the controller.
        
        Additional Features:
        - Integrates Deadband
        - Positive Left

        :returns: The axis value.
        """
        return applyDeadband( -super().getRightX(), self.__deadband )
    
    # Override getLeftTriggerAxis with deadband (for FRC)
    def getLeftTriggerAxis(self) -> float:
        """
        Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the
        range of [0, 1] as opposed to the usual [-1, 1].
        
        Additional Features:
        - Integrates Deadband

        :returns: The axis value.
        """
        return applyDeadband( super().getLeftTriggerAxis(), self.__deadband )
    
    # Override getRightTriggerAxis with deadband (for FRC)
    def getRightTriggerAxis(self) -> float:
        """
        Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the
        range of [0, 1] as opposed to the usual [-1, 1].
        
        Additional Features:
        - Integrates Deadband

        :returns: The axis value.
        """
        return applyDeadband( super().getRightTriggerAxis(), self.__deadband )
    
    # Custom Command to combine Left and Right Trigger Axises
    def getTriggers(self) -> float:
        """
        Get the single axis value of both triggers of the controller. 
        
        Additional Features:
        - Combines LT and RT
        - Integrates Deadband
        - Positive Left

        :returns: The axis value.
        """
        asOneAxis:float = self.getLeftTriggerAxis() - self.getRightTriggerAxis()
        return asOneAxis
    
