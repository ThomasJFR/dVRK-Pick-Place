# MECH 464 Group 1
# This file is the primary file for running the 
# Pick and place operation. It contains the main while loop
# and the system state machine.

# IMPORTS
# Python builtins
from time import time, sleep
from collections import deque
from copy import deepcopy
# Externals
import dvrk
import PyKDL
import numpy as np

# Created
#import SafePSM
from calibration import calibrate_z
from moves import traverse_to_location, move_vertical, home_ecm, safe_retract
from arms import initializePSM, initializeECM 
from vision import main as get_items_from_camera   #capture_system, extract_system_features
from collision import entering_danger_zone, calibrate_collision, collision_risk, mutual_collision_risk
from psm_state_machine import PSMState, StatefulPSM
from utils import FakeWaiter, PSMSequence, dist

items = deque()
bowls = list()
Z_OFFSET = 0.06

def dispatch_item(p):
    """
    This function  interfaces with the computer vision part of the system and decides
    which object to try to pick up next based on which arm is asking, and whether it has
    the mutex.
    """
    global items
    if len(items) == 0:
        return None, True
    # Get the closest target
    item = items.popleft() if (p.name() == "PSM1") else items.pop()
    return item, False

def dispatch_bowl(p):
    global bowls
    if len(bowls) == 0:
        return None, True
    bowl = bowls[0] if (p.name() == "PSM1") else bowls[-1]  # bowls[0] is closest to psm1; bowls[-1] to psm2
    return deepcopy(bowl), False


def mainloop(args):
    # State variables
    psm1, psm2 = args["PSM1"], args["PSM2"]
    #psm1_state, psm2_state = 
    #psm1_state.zcal, psm2_state.zcal = psm1.zcal, psm2.zcal
    psm1_active, psm2_active = True,  True

    start = time()
    lastprint = 0
    move_states = [s for s in PSMState.s if "Move" in s.name]
    while psm1_active or psm2_active:
        now = time()
        psm1_active = False if not psm1_active else not all([psm1.state == PSMState.s.Finished, psm1.waiter and not psm1.waiter.is_busy()])
        psm2_active = False if not psm2_active else not all([psm2.state == PSMState.s.Finished, psm2.waiter and not psm2.waiter.is_busy()])

        if psm1_active:
            danger = entering_danger_zone(psm1)
            #psm1.has_lock = danger and not psm2.has_lock
            if danger:
                if not psm2.has_lock:
                    psm1.has_lock = True
            else:
                psm1.has_lock = False
            
            if not danger or psm1.has_lock:
                psm1.state = tick(psm1)
            else:
                psm1.waiter = FakeWaiter()
            sleep(0.1)
        else:
            psm1.has_lock = False
        
        if psm2_active:
            danger = entering_danger_zone(psm2)
            if danger:
                if not psm1.has_lock:
                    psm2.has_lock = True
            else:
                psm2.has_lock = False
            
            if not danger or psm2.has_lock:
                psm2.state = tick(psm2)
            else:
                psm2.waiter = FakeWaiter()
        else:
            psm2.has_lock = False

            #will_collide = collision_risk(psm1, psm2) and psm2.state in move_states
            #if will_collide:
            #    resolvable = mutual_collision_risk(psm1, psm2) 
            #    psm2.has_lock = resolvable and not psm1.has_lock
            #else:
            #    psm2.has_lock = False
            #
            #if not will_collide or psm2.has_lock:
            #    psm2.state = tick(psm2)
            #else:
            #    psm2.waiter = FakeWaiter()
            sleep(0.1)

        if now - lastprint > 0.5:
            #print ("PSM1: %s\tPSM2: %s" % (
            #    ("!!!" if psm1.has_lock else "") + psm1.state.name, 
            #    ("!!!" if psm2.has_lock else "") + psm2.state.name))
            #print (collision_risk(psm1_state, psm2_state))
            #print (psm1.has_lock, psm2.has_lock)
            if collision_risk(psm1, psm2):
                print "PSM1 Risk:", collision_risk(psm1, psm2, True)
            if collision_risk(psm2, psm1):
                print "PSM2 Risk:", collision_risk(psm2, psm1, True)
            if psm1.has_lock:
                print "PSM1 has lock!"
            if psm2.has_lock:
                print "PSM2 has lock!"
            if not psm1_active:
                print "PSM1 Inactive!"
            if not psm2_active:
                print "PSM2 Inactive!"
            lastprint = now

    return time() - start

def tick(psm):
    if psm.waiter.is_busy():
        return psm.state

    global Z_OFFSEfT
    #
    # STATE MACHINE
    #
    if psm.state == PSMState.s.Standby:
        return PSMState.s.RequestItem

    elif psm.state == PSMState.s.RequestItem:
        psm.target, fault = dispatch_item(psm)
        if fault:
            psm.target= np.array([-0.0261751, 0.135132, -0.18048]) if psm.name == "PSM1" else np.array([0.0203869,-0.170599,-0.50793])
            # EXIT POINT: No more items to collect.
            psm.waiter = safe_retract(psm) 
            return PSMState.s.Finished
        else:
            psm.target[2] = psm.zcal
            print psm.name() + " target: " + str(psm.target)
            return PSMState.s.MoveToItem
    
    elif psm.state == PSMState.s.MoveToItem:
        #print psm.name() + " dist to target:" + str(dist(psm.measured_cp().p, state.target, xy=True))
        if dist(psm.measured_cp().p, psm.target, xy=True) < 0.005:
            return PSMState.s.Grab

        psm.waiter = traverse_to_location(psm, psm.target, Z_OFFSET)
        return psm.state #if state.waiter.is_busy() else PSMState.s.Grab

    elif psm.state == PSMState.s.Grab:
        psm.waiter = PSMSequence([
            psm.jaw.open,
            (move_vertical, (psm, -Z_OFFSET)),
            psm.jaw.close,
            (move_vertical, (psm, +Z_OFFSET)),
        ])
        return PSMState.s.RequestBowl

    elif psm.state == PSMState.s.RequestBowl:
        psm.target, fault = dispatch_bowl(psm)
        if fault:
            print("ERROR! No bowl detected!")
            psm.waiter = psm.safe_retract()
            # EXIT POINT: No bowl detected
            return PSMState.s.Standby
        else:
            print psm.name() + " target: " + str(psm.target)
            psm.target[2] = psm.zcal

            return PSMState.s.MoveToBowl#ApproachBowl
    
    elif psm.state == PSMState.s.ApproachBowl:
        approach_target = np.array([x for x in psm.target])
        approach_target[1] += 0.05 *(1 if psm.name() == "PSM1" else -1)
        
        if dist(psm.measured_cp().p, approach_target, xy=True) <= 0.01:
            return PSMState.s.MoveToBowl

        psm.waiter = traverse_to_location(psm, approach_target, Z_OFFSET)
        return psm.state

    elif psm.state == PSMState.s.MoveToBowl:
        if dist(psm.measured_cp().p, psm.target, xy=True) <= 0.01:
            return PSMState.s.Release
        psm.waiter = traverse_to_location(psm, psm.target, Z_OFFSET)
        return psm.state
   
    elif psm.state == PSMState.s.Release:
        psm.waiter = PSMSequence([
            psm.jaw.open,
            psm.jaw.close
        ])
        # In coppeliasim, PSM1 crashes if we go to the blue bowl.
        # TODO Fix this later
        #if psm.name() == "PSM1":
        return PSMState.s.Standby#BackOffBowl
        #else:
        #    return PSMState.s.BackOffBowl
 
    elif psm.state == PSMState.s.BackOffBowl:
        approach_target = np.array([x for x in psm.target])
        approach_target[1] += 0.05 *(-1 if psm.name() == "PSM1" else 1)
        
        if dist(psm.measured_cp().p, approach_target, xy=True) <= 0.01:
            return PSMState.s.Home

        psm.waiter = traverse_to_location(psm, approach_target, Z_OFFSET)
        return psm.state
    elif psm.state == PSMState.s.Home:
        # state.waiter = psm.home()  # We don't necessarily want this! Let's just get a new target.
        return PSMState.s.RequestItem

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--calibrate", action="store_true", help="Allows user to calibrate the arm before beginning pick-and-place")
    args = parser.parse_args()

    from dvrk import ecm
    e = ecm("ECM")
    #home_ecm(e).wait()

    psm1 = StatefulPSM("PSM1")
    psm2 = StatefulPSM("PSM3")

    # SR: Calibrate the z of the table, if requested
    def calibrate_routine(psm):
        print("------------------------------------------")
        print("  TOUCH %s TIP TO TABLE TO CALIBRATE" % psm.name())
        print("------------------------------------------")
        psm.calibrate()  # Hangs the thread
        print("Calibrated at ", psm.setpoint_cp().p) 
    
    # FOR NOW, FORCE CAL
    #psm1_zc = calibrate_routine(psm1) if args.calibrate else psm1.setpoint_cp().p[2]
    #psm2_zc = calibrate_routine(psm2) if args.calibrate else psm2.setpoint_cp().p[2]
    args.calibrate = True
    calibrate_routine(psm1)
    w1 = safe_retract(psm1)
    calibrate_routine(psm2)
    w2 = safe_retract(psm2)
    while any([w1.is_busy(), w2.is_busy()]):
        continue

    #x, y  = get_items_from_camera()
    x = [0.0657, 0.05340, -0.0383, -0.0359]
    y = [0.0370, -0.1191, 0.00322, -0.0741]
    z = [0] * len(x)
    triplets = list(zip(x,y,z))
    features = {
        "items": [np.array(triplet) for triplet in triplets
            #np.array([0,0,0.01]),
            #np.array([0.02,-0.03,0.01]),
            #np.array([0.03,0.02,0]),
            #np.array([-0.05,-0.1,0.01]),
            #np.array([0,-0.1,0]),
        ],
        "bowls": [
            np.array([0.08,-0.05, 0])
            #np.array([-0.07,0.05, 0]),
            #np.array([0.08,-0.16, 0]),
        ]
    }
    # SR: Generate features
    # Sort the items and bowls by their positions such that
    # items closer to psm1 appear on the left side of the list.
    # TODO Fix the bad global pattern
    sort_by_y = lambda xyz: -xyz[1]
    items = list(sorted(features["items"], key=sort_by_y))
    bowls = list(sorted(features["bowls"], key=sort_by_y))
    if False:#args.calibrate:
        for i in range(len(items)):
            items[i][2] = z_calibrated
        for i in range(len(bowls)):
            bowls[i][2] = z_calibrated 
    items = deque(items)
    
    calibrate_collision(bowls[0][1])

    # Let the games begin!
    psm1.trajectory_j_set_ratio(0.2)
    psm2.trajectory_j_set_ratio(0.2)
    elapsed = mainloop({
        "PSM1": psm1,
        "PSM2": psm2,
    })
    print("Task finished. Elpsed time: %ds" % elapsed)

