from transitions import Machine

states = ['RENDEZVOUS', 'MOVING']  
transitions = [
    {'trigger': 'MOVE', 'source': 'RENDEZVOUS', 'dest': 'MOVING'},
    {'trigger': 'STOP', 'source': 'MOVING', 'dest': 'RENDEZVOUS'}
] 

class MyStateMachine(object):
    def __init__(self):
        self.machine = Machine(model=self, states=states, transitions=transitions, initial='RENDEZVOUS')



