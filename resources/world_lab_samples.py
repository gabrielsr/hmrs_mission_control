import pytest

from enum import Enum
from mission_control.mission.ihtn import Method, MethodOrdering, Task, ElementaryTask, AbstractTask
from mission_control.core import POI

class task_type(Enum):
    NAV_TO = 'navigation'
    APPROACH_PERSON = 'approach_person'
    AUTHENTICATE_PERSON = 'authenticate_person'
    OPERATE_DRAWER = 'operate_drawer'
    APPROACH_ROBOT = 'approach_robot'
    PICK_UP = 'pick_up'
    DEPOSIT = 'deposit'

all_skills = [ tt.value for tt in task_type ]

class poi(Enum):
    respiratory_control = POI("Respiratory Control")
    ic_corridor = POI("IC Corridor")
    pc_corridor = POI("PC Corridor")
    ic_room_1 = POI("IC Room 1")
    ic_room_2 = POI("IC Room 2")
    ic_room_3 = POI("IC Room 3")
    ic_room_4 = POI("IC Room 4")
    ic_room_5 = POI("IC Room 5")
    ic_room_6 = POI("IC Room 6")
    pc_room_1 = POI("PC Room 1")
    pc_room_2 = POI("PC Room 2")
    pc_room_3 = POI("PC Room 3")
    pc_room_4 = POI("PC Room 4")
    pc_room_5 = POI("PC Room 5")
    pc_room_6 = POI("PC Room 6")
    pc_room_7 = POI("PC Room 7")
    pc_room_8 = POI("PC Room 8")
    pc_room_9 = POI("PC Room 9")
    pc_room_10 = POI("PC Room 10")
    reception = POI("Reception")
    pharmacy_corridor = POI("Pharmacy Corridor")
    pharmacy = POI("Pharmacy")

all_rooms = [ poi_.value for poi_ in [ poi.ic_room_1, poi.ic_room_2, poi.ic_room_3, poi.ic_room_4, 
              poi.ic_room_5, poi.ic_room_6, poi.pc_room_1, poi.pc_room_2, 
              poi.pc_room_3, poi.pc_room_4, poi.pc_room_5, poi.pc_room_6, 
              poi.pc_room_7, poi.pc_room_8, poi.pc_room_9, poi.pc_room_10 ]]

class Roles(Enum):
    nurse =  'nurse'
    lab_arm = 'lab_arm'
    r1 = 'r1'

# Defined as Enum so we can reference methods and tasks, and we can have references
# to names that we later on set on them with set_name()


def pickup_ihtn(pickup_location):
    class lab_samples_ihtn(Enum):
        # elementary tasks
        navto_room = ElementaryTask(task_type.NAV_TO, destination=pickup_location, assign_to=[Roles.r1])
        approach_nurse = ElementaryTask(task_type.APPROACH_PERSON, target=Roles.nurse, assign_to=[Roles.r1])
        authenticate_nurse = ElementaryTask(task_type.AUTHENTICATE_PERSON, target=Roles.nurse, assign_to=[Roles.r1])
        open_drawer = ElementaryTask(task_type.OPERATE_DRAWER, action='open', assign_to=[Roles.r1])
        deposit = ElementaryTask(task_type.DEPOSIT, assign_to = [Roles.nurse])
        close_drawer = ElementaryTask(task_type.OPERATE_DRAWER, action='close', assign_to=[Roles.r1])
        navto_pharmacy = ElementaryTask(task_type.NAV_TO, destination=poi.pharmacy.value, assign_to=[Roles.r1])
        approach_arm = ElementaryTask(task_type.APPROACH_ROBOT, target=Roles.lab_arm, assign_to=[Roles.r1])
        open_drawer_lab = ElementaryTask(task_type.OPERATE_DRAWER, action='open', assign_to=[Roles.r1])
        pick_up_sample  = ElementaryTask(task_type.PICK_UP, target=Roles.r1, assign_to=[Roles.lab_arm])
        close_drawer_lab = ElementaryTask(task_type.OPERATE_DRAWER, action='close', assign_to=[Roles.r1])


        # methods and abstract tasks
        m_deposit = Method(subtasks = [open_drawer, deposit, close_drawer])
        deposit_sample_on_delivery_bot = AbstractTask(methods=[m_deposit])
        m_retrieve = Method(subtasks = [approach_nurse, authenticate_nurse, deposit_sample_on_delivery_bot])
        retrive_sample = AbstractTask(methods=[m_retrieve])
        m_unload = Method(subtasks=[open_drawer_lab, pick_up_sample, close_drawer_lab])
        unload_sample = AbstractTask(methods=[m_unload])
        m_mission = Method(subtasks=[navto_room, retrive_sample, navto_pharmacy, approach_arm, unload_sample])

        # root task
        pickup_sample = AbstractTask(methods=[m_mission])
        
    for enum_item in lab_samples_ihtn:
        setattr(enum_item.value, 'name', enum_item.name)

    return (lab_samples_ihtn.pickup_sample.value, lab_samples_ihtn)


pickup_sample, lab_samples_ihtn =  pickup_ihtn(poi.ic_room_3.value)


@pytest.fixture
def ihtn_pickup_sample():
    return pickup_sample

@pytest.fixture
def ihtn_unload_sample():
    return lab_samples_ihtn.unload_sample.value

@pytest.fixture
def ihtn_navto_room3():
    return lab_samples_ihtn.navto_room.value

@pytest.fixture
def ihtn_deposit():
    return lab_samples_ihtn.deposit.value