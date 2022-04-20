from typing import Mapping


class OnEachEnsemble:
    pass


class EnvChangeHandler():
    def on_env_change(self):
        return Chain[self.analyse_context_change, self.plan_for_env_change, self.execute_mr_for_env_change]

    def on_mission_change(self):
        pass

    def analyse_context_change(self, env_change):
        ''' check if the env_change affects the active and future tasks '''
        return  None# it does not affect

    def plan_for_env_change(self, blocked_task):
        ''' try repair mission '''

        ''' '''
        pass

class EnvChangeEvent:
    pass

class MissionCoordintor:
    ''' execute coordination processes:
        - env change adaptation process '''
    
    def __init__(self, request):
        self.env_adaptation_process = EnvChangeAdaptationProcess()
        #self.add_process(self.mission_management_process(self.requests))
        #self.mission_management_process = EnvChangeAdaptationProcess()
        self.global_plan = request.plan
        local_plans: Mapping = localize(self.global_plan)


        # processes
        eca_process = EnvChangeAdaptationProcess()
        eca_process.set_mission(global_plan)

        # how to divide cf process between the coordinator and the ensemble? 
        cf_process = CoalitionFormationProcess()
        cf_process.set_mission(global_plan)

        supervision_process = SupervisionProcess()
        supervision_process.set_mission(global_plan)

    def on_join(self, ensemble):
        if ensemble is mission_coalition:
            self.local_plans.for_each(init_worker_role)


    def init_worker_role(role_key, local_plan):
        def on_assigned(worker):
            pass

        role = Role(role_key, on_join=on_assigned, local_plan)


    def setup(self):
        mission_server: MissionServer = self.parent
        mission_server.register('on_env_change', self.event_change_handler.on_env_change())
    




    def listen_to(self, register):
        register.on(EnvChangeEvent, self.env_adaptation_process.monitor)


    def get_env_adaptation_process(self):
        self.env_adaptation_process
        

class EnvChangeAdaptationProcess(Process):
    def on_ensemble_start(self, global_plan):
        # init knowledge
        self.knowledge = None 
        # create a loop
        self.adaptation_loop = Loop(self.monitor, self.analyse, self.process, self.execute)

    def update_knowledge_global_pla(self, global_plan):
        self.knowledge.global_plan = global_plan

    def monitor(self, env_change_event):
        ''' check if the env change is of interest '''
        if not is_event_change_monitored(env_change_event):
            return None

        pass

    def analyse(self):
        pass

    def process(self):
        pass

    def execute(self):
        pass

    def is_event_change_monitored(self, env_change_event):
        pass

class MissionCoalition(Ensemble[MissionCoordinator]):
    ''' Realize the coalition formation'''
    
    def __init__(self, request, user, coordinator, env_monitor_coordinator):
        self.subscription_type(UtilitySubscription) # any candiddate that meet the membership condition


    def membership(self, component):
        ''' check restrictions 
            i.e., it componenent satisfy any role on the local_plans ''' 
        pass

    def utility(self, curr_state, candidate):
        ''' check optimization objectives
            i.e., make and select bids '''
        return # inverse of required time



class EnvSensor(Component, Publisher):
    def __init__(self):
        self.publisher_types.add(EnvChangeEvent)

    @process
    def sense():
        if x:
            event = EnvChangeEvent()
            self.publish(event)


class EnvSensing(Ensemble):
    def __init__(self):
        self.subscription_type(OpenSubscription) # any candiddate that meet the membership condition
        # within ensemble roles
        sensor_role = Role(Sensor, on_join=self.on_sensor_join)
        mission_coordinator_role  = Role(MissionCoordintor, on_join=self.on_join_mc)
        
        # init knowledge exchange channels
        self.env_change_envets_broker = PubSubBroker()


    # n by m subscription. Any sensor publish,
    def on_join_sensor(self, sensor):
        sensor.subscribe(EnvChangeEvent, self.broker.broke)

    def on_join_mc(self, mission_coordinator):
        self.broker.subscribe(EnvChangeEvent, mission_coordinator)
        

    def membership(self, candidate):
        ''' Any superensemble member, that is of type role '''
        if sensor_role.check(candiate) or mission_coordinator_role.check(candidate):
            return True
        pass

        
class MissionServer(Ensemble):
    
    def __init__(self, env_monitor):
        super.__init__(subscription_type = OpenSubscription)
        self.user_role = Role(User, on_join=self.on_join_user)
        self.worker_role = Role(Worker)
        

        self.requests = Subscribable()

    def membership(self, component):
        ''' Any component of type robot or user '''
        return component is within_roles([self.user_role, self.worker_role])

    def subensembles(self):
        # multiple ensembles
        pass        

    def setup(self):
        # singleton
        pass
    
    def on_join_user(self, user):
        user.subscribe('request', create_mission_subensemble(user))

    def create_mission_subensemble(self, user):
        
        def _(request):
            mission_coordinator = init_service_node(component=MissionCoordintor)
            mission_coalition = init_subensemble(self, mission_coordinator, MissionCoalition)
        
        return _


class coalition_management_process(self) -> Ensemble.Process:
    ''' process of management of coalitions
        For each new request a coordinator, and an ensemble is created '''
    
    def create_mission(self, request):
        # instantiate a coordinator
        mission_coordinator = self.new_service_component(MissionCoordintor, request)
        mission_coordinator = 

        # end of mission handler
        mission_coordinator.on('mission_end', self.end_of_mission)
        
        # bind to subensembles
        self.subensembles.add()

    def on_end_of_mission(self):
        self.mission_server.requests.remove()

    return Ensemble.Process(setup)


class Root(Ensemble):

    def __init__(self):
        pass

    
    def setup(self):
        # setup new components discovery
        self.disover_new_components()


    def subensembles(self):
        # singleton
        yield SingletonEnsemble(EnvSensingCoordination)
        mission_server = SingletonEnsemble(MissionServer)
        yield mission_server

    
    def disover_new_components(self):
        pass

    