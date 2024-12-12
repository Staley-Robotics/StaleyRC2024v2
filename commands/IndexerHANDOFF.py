import typing

from commands2 import Command, Subsystem
from subsystems.Indexer import Indexer, IndexerSpeeds

class IndexerHANDOFF(Command):
    # Variable Declaration
    m_subsystem:Indexer = None
    m_getValue:typing.Callable[[],float] = lambda: 0.0
    
    # Initialization
    def __init__( self,
                  mySubsystem:Subsystem,
                ) -> None:
        # Command Attributes
        self.m_subsystem:Indexer = mySubsystem
        self.speeds = IndexerSpeeds() # maybe put all commands into one file? probably not... idk
        self.m_getValue = self.speeds.HANDOFF
        self.setName( "SampleCommand" )
        self.addRequirements( mySubsystem )

    # On Start
    def initialize(self) -> None:
        pass

    # Periodic
    def execute(self) -> None:
        self.m_subsystem.setSpeed( self.m_getValue )
        self.m_subsystem.m_logging.putString('Indexer Status', 'HANDOFF')

    # On End
    def end(self, interrupted:bool) -> None:
        pass

    # Is Finished
    def isFinished(self) -> bool:
        return self.m_subsystem.hasHalfNote() == -1 
        # Logic: if only upper sensor sees note, then handoff is finished... ok makes sense

    # Run When Disabled
    def runsWhenDisabled(self) -> bool:
        return False