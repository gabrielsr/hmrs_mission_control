from typing import Final

from ..core import SkillDescriptor
from .routes_ed import RoutesEnvironmentDescriptor
from ...core import POI, Capability, Estimate, ImpossibleToEstimate

class Move(Capability):
    def __init__(self, avg_speed, u):
        super().__init__(avg_speed= avg_speed, u=u)

class NavigationSkillDescriptor(SkillDescriptor):
    # skill required capability


    name: Final = 'navigate'
    required_capabilities: Final = [Move]

    def __init__(self, routes_ed: RoutesEnvironmentDescriptor):
        self.routes_ed: RoutesEnvironmentDescriptor = routes_ed
    
    def estimate(self, task_context):
        origin: POI = task_context.get('origin')
        dest: POI = task_context.get('destination')
        avg_speed = task_context.worker.get('avg_speed')

        route = self.routes_ed.get(origin, dest)
        if not route:
            return ImpossibleToEstimate(reason=f'No route from {origin} to {dest}')
        distance = route.get_distance()
        estimate_time = distance / avg_speed
        return Estimate(time = estimate_time)
