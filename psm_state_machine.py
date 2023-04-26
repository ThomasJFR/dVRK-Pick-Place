from math import radians
from enum import Enum

from utils import FakeWaiter
from dvrk import psm as PSM
import numpy as np

from copy import deepcopy

class PSMState:
    s = Enum("PSM States", [
        "Standby",
        "RequestItem",
        "MoveToItem", 
        "Descend",
        "Grab",
        "Ascend",
        "RequestBowl",
        "ApproachBowl",
        "MoveToBowl",
        "Release",
        "BackOffBowl",
        "Home",
        "Finished"
    ])
    
    def __init__(self):
        self.state = PSMState.s.Standby 
        self.target = None
        self.waiter = FakeWaiter()
        self.has_lock = False
        

class StatefulPSM(PSM, PSMState):
    def __init__(self, arm_name, autostart=True):
        PSM.__init__(self, arm_name)
        PSMState.__init__(self)

        self.zcal = None  # Use to calibrate Z

        if autostart:
            self.start()

    def start(self):
        self.enable()
        self.home()

        self.trajectory_j_set_ratio(0.5)
        return FakeWaiter() #self.safe_retract()

    def is_calibrated(self):
        return self.zcal is not None

    def calibrate(self, from_current=False):
        """
        Performs a Z-calibration of the gripper.
        if from_current is False, the program will hang until the enter key is pressed.
        This allows the operator to perform the calibration
        """
        if not from_current:
            # Hang the program while the user adjusts the arm
            raw_input("Press enter to continue")

        # Extract arm position
        self.zcal = deepcopy(self.measured_cp().p[2])

    def safe_retract(self):
        """
        Retracts the extender, then moves to the vertical position.
        """
        current = p.setpoint_jp()
        sequence = []

        align_gripper = deepcopy(current)
        align_gripper[4] = align_gripper[5] = 0
        sequence.append(self.move_jp, (align_gripper,))

        retract_arm = deepcopy(align_gripper)
        retract_arm[2] = 53.5 * 1E-3
        sequence.append(self.move_jp, (retract_arm,))
        
        align_arm = deepcopy(retract_armt)
        align_arm[0] = align_arm[1] = align_arm[3] = 0
        sequence.append(self.move_jp, (align_arm,))

        return PSMSequence(sequence)

    def open(self, angle=radians(60)):
        """
        Opens the jaws fully or by the specified amount in radians
        Unstickies the jaws too (in CoppSim)
        """
        return self.jaw.open(angle)

    def close(self):
        """
        Closes the jaws and makes them sticky
        """
        return self.jaw.close()

    def move_dcp(self, dx=0, dy=0, dz=0):
        """
        Move the end-effector in cartesian space by a specified amount 
        relative to the current cartesian position. 
        """
        target = self.setpoint_cp()
        #target.p[0] += dx
        #target.p[1] += dy
        #target.p[2] += dz
        target.p += np.array([dx, dy, dz])
        return self.move_cp(target)
   
    def safe_move_pos(self, x=None, y=None, z=None, aggressive=True):        
        target = self.setpoint_cp()
        target.p[0] = target.p[0] if x is None else x
        target.p[1] = target.p[1] if y is None else y
        target.p[2] = target.p[2] if z is None else z
        if self.in_safezone(target.p):  # Move is safe! 
            return self.move_cp(target)
        
        # If the move was not safe, determine what to do
        print("Requested move is beyond the safezone!")
        if aggressive:
            new_x, new_y, new_z = self.constrain_to_safezone(target.p)
            target.p[0] = new_x
            target.p[1] = new_y
            target.p[2] = new_z 
            return self.move_cp(target)
        else:
            return FailWaiter()

    def safe_move_dpos(self, dx=0, dy=0, dz=0, aggressive=True):
        """
        As with move_dcp, but only works if the movement is within the safezone.
        If aggressive is True, the arm will go as close to its final position as it can.
        """
        x, y, z = self.setpoint_cp().p
        return self.safe_move_pos(x + dx, y + dy, z + dz)

# Helpers to make the acquisiton process quicker
def SafePSM1(): return SafePSM("PSM1")
def SafePSM2(): return SafePSM("PSM2")
def SafePSM3(): return SafePSM("PSM3")       

