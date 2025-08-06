


# Transitions allows for state machines to be graphed if a plugin is installed and then a graph version of the state machine is used.
CAN_GRAPH_STATES = False
try:  # pragma: no cover
    import graphviz  # pragma: no flakes
    from transitions.extensions.factory import HierarchicalGraphMachine as HierarchicalMachine
    CAN_GRAPH_STATES = True
except ImportError:  # pragma: no cover
    from transitions.extensions.factory import HierarchicalMachine as HierarchicalMachine
from transitions.extensions.states import add_state_features, Timeout

# A custom hierarchical state machine for the generator.
# This is done so we can add automatic timeouts to some states which cause a raised transition to occur.
@add_state_features(Timeout)
class CustomHierarchicalMachine(HierarchicalMachine):
    pass
