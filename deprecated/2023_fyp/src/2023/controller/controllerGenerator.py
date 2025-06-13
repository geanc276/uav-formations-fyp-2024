#!/usr/bin/env python3
import sys
import json


TAB = '\t'  # allows for f string expressions involving the tab character to be evaluated


def generateImportStatements():
    shebang = '#! /usr/bin/env python'
    imports = 'import rospy\nimport sys\nfrom std_msgs.msg import Header\nfrom geometry_msgs.msg import PoseStamped, ' \
              'TwistStamped\nfrom sensor_msgs.msg import NavSatFix\nfrom std_msgs.msg import String, Float64, ' \
              'Bool\nfrom mavros_msgs.msg import State\nfrom mavros_msgs.srv import CommandBool, CommandBoolRequest, ' \
              'SetMode, SetModeRequest\nimport json\nimport statemachine\nimport rospysimple\n'
    return shebang + '\n' + imports + '\n\n\n'


def generateEFSMClass():
    return 'class ' + 'DroneMissionMachine' + '(statemachine.StateMachine):\n'


def generateStateDeclarations(states):
    state_string = '\t'
    
    for i in range(len(states)):
        state_string += states[i]['name'] + ' = statemachine.State('
        if states[i]['position'] == 'initial':
            state_string += 'initial=True'
        elif states[i]['position'] == 'final':
            state_string += 'final=True'
        state_string += ')\n' + '\t'
    
    return state_string + '\n'


def generateTransitionDeclarations(transitions):
    transition_string = '\t'
    for i in range(len(transitions)):
        transition_string += transitions[i]['event'] + ' = ' + transitions[i]['origin'] + '.to(' + \
                             transitions[i]['destination']
        transition_string += ')\n' + '\t'
    
    return transition_string + '\n'


def generateEventGenerator(events):
    eventGeneratorString = '\tdef eventGenerator(self):\n'
    eventGeneratorString += '\t\tif ' + events[0]['trigger'] + ':\n'
    eventGeneratorString += '\t\t\tself.' + events[0]['name'] + '()\n'
    for i in range(len(events) - 1):
        eventGeneratorString += '\t\telif ' + events[i + 1]['trigger'] + ':\n'
        eventGeneratorString += '\t\t\tself.' + events[i + 1]['name'] + '()\n'

    return eventGeneratorString + '\n'


def generateUserDeclarations(user_declarations_file_name):
    userDeclarationsString = ''
    with open(user_declarations_file_name, 'r') as ud:
        for line in ud:
            userDeclarationsString += '\t' + line
    return userDeclarationsString + '\n\n'


def generateMain(mission_name, drone_name, event_generator_rate_hz):
    mainString = 'def main():\n\t' + mission_name + ' = DroneMissionMachine()\n\t' + \
        'rospy.init_node("' + drone_name + '_life_cycle")\n\t'\
                 'event_loop = rospy.Rate(' + f"{event_generator_rate_hz}" + ')\n\t' +\
                 'while not rospy.is_shutdown():\n\t\t' + mission_name + '.eventGenerator()\n\t\tevent_loop.sleep()\n'
    mainString += '\nif __name__ == "__main__":\n\tmain()'
    return mainString


def readUserSpecification(specification_file_name):
    with open(specification_file_name, 'r') as spec:
        spec_object = json.load(spec)
    return spec_object


def main():
    user_specifications_file_name = sys.argv[1]
    user_declarations_file_name = sys.argv[2]
    output_file_name = sys.argv[3]
    drone_name = sys.argv[4]
    userSpecifications = readUserSpecification(user_specifications_file_name)
    generatedCode = ''
    generatedCode += generateImportStatements()
    generatedCode += generateEFSMClass()
    generatedCode += generateStateDeclarations(userSpecifications['states'])
    generatedCode += generateTransitionDeclarations(userSpecifications['transitions'])
    generatedCode += generateEventGenerator(userSpecifications['events'])
    generatedCode += generateUserDeclarations(user_declarations_file_name)
    generatedCode += generateMain(userSpecifications['mission_name'], drone_name,
                                  userSpecifications['event_generator_rate_hz'])

    with open(output_file_name, 'w') as gc:
        gc.write(generatedCode)
    

if __name__ == "__main__":
    main()
