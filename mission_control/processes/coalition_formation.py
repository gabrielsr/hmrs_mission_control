
from typing import Generator, Dict, List, Sequence

from mission_control.mission.ihtn import ElementaryTask, Task
from mission_control.mission.planning import distribute, flat_plan
from .integration import MissionHandler, MissionUnnexpectedError
from ..core import MissionContext, Worker, LocalMission, Role
from ..estimate.estimate import EstimateManager, Bid

def coalitionFormationError(e, mission_context):
    message = f'Unexpected error forming coalition for {mission_context}'
    return MissionUnnexpectedError(e, message)



class CoalitionFormationProcess:
    """ Service of creating coalitions. It receives an ihtn with tasks 
    assigned to roles an return a selection of robots to execute the plan """

    def __init__(self, estimate_manager: EstimateManager):
        self.individual_plans = []
        self.estimate_manager: EstimateManager = estimate_manager
        

    def run(self, mission_context: MissionContext, workers: List[Worker], mission_handler: MissionHandler):
        #try:
        self.do_run(mission_context, workers, mission_handler)
        # except Exception as e:
        #     mission_handler.handle_unnexpected_error(coalitionFormationError(e, mission_context))

    def do_run(self, mission_context: MissionContext, workers: List[Worker], mission_handler: MissionHandler):
        if mission_context.status == MissionContext.Status.NEW:
            mission_context.local_missions = list(self.initialize_local_missions(mission_context))
            mission_context.state = MissionContext.Status.PENDING_ASSIGNMENTS
            mission_handler.start_mission(mission_context)

        is_success = self.create_coalition(mission_context, workers)
        if is_success:
            mission_context.status = MissionContext.Status.DISTRIBUTING_TASKS
            mission_handler.update_assigments(mission_context)
        else:
            # its all or northing - or assign all pending tasks or none
            mission_handler.no_coalition_available(mission_context)

    def create_coalition(self, mission_context: MissionContext, workers: List[Worker]) -> bool:
        plan_rank_map = {}
        for local_mission in self.get_pending_assignments(mission_context):
            task_list = self.flat_plan(local_mission.plan)
            bids = []
            candidates = self.get_compatible_workers(task_list, workers)
            for worker in candidates:
                bid = self.estimate(worker, task_list)
                is_viable = self.check_viable(bid)
                if is_viable:
                    bids.append(bid)
                else:
                    pass # TODO log
            if not bids: # empty list of viable bids
                mission_context.occurances.append(f'no viable assignment for {local_mission.role}')
                return False
            bids = self.rank_bids(bids)
            plan_rank_map[local_mission] = bids

        selected_bids =  self.select_bids(plan_rank_map)
        self.set_assignment_from_selected_bids(selected_bids)
        return True

    @staticmethod    
    def get_pending_assignments(mission_context: MissionContext) -> Sequence[LocalMission]:
        return filter(lambda lm: lm.status == LocalMission.Status.PENDING_ASSIGNMENTS, 
                        mission_context.local_missions)

    @staticmethod
    def initialize_local_missions(mission_context: MissionContext) -> Generator[LocalMission, None, None]:
        for role in mission_context.global_plan.assign_to:
            local_mission = distribute(mission_context.global_plan, role)
            lm = LocalMission(local_plan=local_mission, role=role, global_mission = mission_context)
            if role.type == Role.Type.NOT_MANAGED:
                lm.status = LocalMission.Status.NOT_MANAGED
            yield lm
        return

    @staticmethod
    def flat_plan(task) -> List[ ElementaryTask ]:
        return flat_plan(task)

    def get_compatible_workers(self, task_list: List[ElementaryTask], workers: List[Worker]):
        """  get the workers that have the required skills for executing all tasks in 'task_list' """
        required_skills = set([ task.type for task in task_list if isinstance(task, ElementaryTask)])
    
        for worker in workers:
            if not required_skills.difference(worker.skills):
                yield worker
        return
        
    def estimate(self, worker, task_list: List[ElementaryTask]) -> Bid: 
        return self.estimate_manager.estimate(worker, task_list)

    @staticmethod
    def check_viable(bid: Bid) -> bool:
        if bid.estimate.is_impossible_to_estimate:
            return False
        # TODO check worker resources / battery
        else:
            return True
    @staticmethod
    def rank_bids(bids: List[Bid]) -> List[Bid]:
        return sorted(bids, key=lambda bid: bid.estimate.time, reverse=False)

    @staticmethod
    def select_bids(plan_rank_map: Dict):
        selected_bids = {}
        for local_mission, bid_rank in plan_rank_map.items():
            #TODO estimate the wait time and verify resources
            selected_bids[local_mission] = bid_rank[0]
        return selected_bids
    
    def set_assignment_from_selected_bids(self, selected_bids):
        for local_mission, bid in selected_bids.items():
            local_mission.worker = bid.worker
            local_mission.status = LocalMission.Status.PENDING_COMMIT
            self.set_plans_into_tasks(local_mission, bid.partials)
    
    @staticmethod
    def set_plans_into_tasks(local_mission, partials):
        for partial in partials:
            if partial.plan is not None:
                partial.task.plan = partial.plan