from PyKDL import Vector
from psm_state_machine import PSMState
from utils import dist, dist_from_line

y_boundary = 0

def calibrate_collision(ypos):
    global y_boundary
    y_boundary = ypos

def mutex_collision_risk():
    pass

def entering_danger_zone(psm):
    if psm.target is None:
        return False

    global y_boundary
    pos, tgt = psm.measured_cp().p, Vector(*psm.target)
    return any([tgt.y() - 0.01 < y_boundary if psm.name() == "PSM1" else tgt.y() + 0.01 > y_boundary,
        pos.y() - 0.01 < y_boundary if psm.name() == "PSM1" else pos.y() + 0.01 > y_boundary])

def new_collision_risk(psm):
    if psm.target is None:
        return [False] if as_list else False

    global y_boundary
    pos, tgt = psm.measured_cp().p, Vector(*psm.target)
    conditions = [
        tgt.y() - 0.03 < y_boundary if psm.name() == "PSM1" else tgt.y() + 0.03 > y_boundary
    ]

def collision_risk(psm1, psm2, as_list=False):
    if psm1.target is None or psm2.target is None:
        return [False] if as_list else False


    # Extract positions and targets
    pos1, tgt1 = psm1.measured_cp().p, Vector(*psm1.target)
    pos2, tgt2 = psm2.measured_cp().p, Vector(*psm2.target)
    
    nearby = lambda p1, p2: dist(p1, p2, xy=True) < 0.02
    on_route = lambda P, linepoints: dist_from_line(P, *linepoints) < 0.02

    danger_states = [PSMState.s.MoveToItem, PSMState.s.MoveToBowl]
    conditions = [
        #nearby(tgt1, tgt2),  # PSM targets are overlapping
        #nearby(pos2, tgt1),  # The other PSM is on top of the target
        #nearby(pos1, tgt2),  # The other PSM is on top of the target
        #on_route(pos2, (pos1, tgt1)),  # The other PSM is between us an the target
        #on_route(pos1, (pos2, tgt2)),  # The other PSM is between us an the target
        tgt1.y() - 0.01 < tgt2.y(),
        tgt1.y() - 0.01 < pos2.y(),
        pos1.y() - 0.01 < tgt2.y()
    ]
    global y_boundary
    conditions = [
        tgt1.y() < y_boundary,
        tgt2.y() > y_boundary
    ]
    return conditions if as_list else any(conditions)

def mutual_collision_risk(psm1, psm2, as_list=False):
    return collision_risk(psm1, psm2, as_list)

    if psm1.target is None or psm2.target is None:
        return [False] if as_list else False

    # Extract positions and targets
    pos1, tgt1 = psm1.measured_cp().p, Vector(*psm1.target)
    pos2, tgt2 = psm2.measured_cp().p, Vector(*psm2.target)
    
    nearby = lambda p1, p2: dist(p1, p2, xy=True) < 0.02
    on_route = lambda P, linepoints: dist_from_line(P, *linepoints) < 0.02

    conditions = [
        #nearby(tgt1, tgt2),  # PSM targets are overlapping 
        tgt1.y() - 0.01 < tgt2.y(),
        tgt1.y() - 0.01 < pos2.y(),
        pos1.y() - 0.01 < tgt2.y()
    ]
