from typing import List, Set


class ComponentRoles:
    """ Each component role r ε R is associated with a function rdom(VP) that for a given valuation of 
        ensemble parameters VP determines the component instances that may be selected 
        for the role (i.e., the powerset 2rdom (V p ) is the domain for the role r). """
    
    

class SubEnsembleGroups:
    pass

class Ensemble:

    def __init__(self, parameters, roles: list, ):
        self.parameters = parameters #  P is a set of ensemble parameters
        self.roles: Set[ComponentRoles] = None # R is a set of component roles in E;
        self.groups: Set[SubEnsembleGroups] = None # G is a set of sub-ensemble groups in E;


    def membership_condition(): # M is a membership condition
        pass

    def utility_function(): # U is a utility function;
        pass

    def task_function(): # T is a task function.
        pass

