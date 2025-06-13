#!/usr/bin/env python3
import sys
import json

# Working EFSM code generation for generic platform implementation
# User
#

TAB = "\t"  # allows for f string expressions involving the tab character to be evaluated


# Creates the standard preamble for the C++ source. Should be extended to allow for user specified library extended (need to specify this in JSON format first)
def createImplementationIncludeString(userDefinitions=""):
    includeString = "#include <iostream>\n#include \"generatedDefinitions.h\"\n"
    if (userDefinitions):
        includeString += f"#include \"{userDefinitions}\"\n\nusing namespace std;\nusing namespace generatedDefinitions;\nusing namespace {userDefinitions[:-2]};\n\n"
    else:
        includeString += "\nusing namespace std;\nusing namespace generatedDefinitions;\n\n"

    return includeString

def createDefinitionIncludeString():
    definitionIncludeString = "#pragma once\n#include <iostream>\n\nusing namespace std;\n\nnamespace generatedDefinitions {\n"
    return definitionIncludeString

# Creates the type definition for the state enum
def createStateString(states):
    stateString = "\ttypedef enum State {\n"
    for i in range(len(states)):
        stateString += f"{TAB*2}{states[i]}"
        if i < (len(states) - 1):
            stateString += ",\n"
        else:
            stateString += "\n\t} State_t;\n\n"
    return stateString


# Creates the type definition for the event enum (note that the placeholder state NONE is automatically included as the first state)
def createEventString(events):
    eventString = f"\ttypedef enum Event {{\n{TAB*2}NONE,\n"
    
    for i in range(len(events)):
        eventString += f"{TAB*2}{events[i]}"
        if i < (len(events) - 1):
            eventString += ",\n"
        else:
            eventString += "\n\t} Event_t;\n}\n\n"
    
    return eventString


# Creates the global variable declarations and init() definition (which includes initialisation of state and user variables)
def createInitString(variables, initialState):
    initString = f"State_t currentState = {initialState};\n"
    
    for i in range(len(variables)):
        initString += f"{variables[i]['type']} {variables[i]['name']} = {variables[i]['initialValue']};\n"
    initString += "\n"

    return initString


# Creates the definition of the tran() function (which just changes the state to the state passed in the argument)
def createTranString():
    tranString = "void tran(State_t nextState) {\n\tcurrentState = nextState;\n}\n\n"
    return tranString


# Creates the definition of the dispatch() function 
# which uses the nested switch format (ie states -> events -> transitions -> (guards if applicable) -> actions).
# Requires a list of actions with at least one element. Guards are optional.
# Multiple transitions from the same state triggered by the same event are supported, but user is required to ensure determinism
def createDispatchString(states, events, transitions):
    dispatchString = "void dispatch(Event_t event) {\n\tswitch(currentState) {\n"
    
    for i in range(len(states)):
        dispatchString += f"{TAB*2}case {states[i]}:\n{TAB*3}switch(event) {{\n"
        
        for j in range(len(events)):
            dispatchString += f"{TAB*4}case {events[j]}:\n"
            duplicateEventTransition = False
            for k in range(len(transitions)):
                if ((transitions[k]['fromState'] == states[i]) and (transitions[k]['event'] == events[j])):
                    # Try for guard conditions, if an exception occurs assume it is due to the absence of a guard (reasonable as JSON parser will catch JSON syntax errors)
                    try:
                        if (duplicateEventTransition == False):
                            duplicateEventTransition = True      
                            dispatchString += f"{TAB*5}if ({transitions[k]['guard']}) {{\n"      
                        else:
                            dispatchString += f"{TAB*5}else if ({transitions[k]['guard']}) {{\n"
                            
                        for l in range(len(transitions[k]['actions'])):
                            dispatchString += f"{TAB*6}{transitions[k]['actions'][l]};\n"
                            
                        dispatchString += f"{TAB*6}tran({transitions[k]['toState']});\n"
                        dispatchString += f"{TAB*5}}}\n"
                        
                    except:
                        for l in range(len(transitions[k]['actions'])):
                            dispatchString += f"{TAB*5}{transitions[k]['actions'][l]};\n"
                            
                        dispatchString += f"{TAB*5}tran({transitions[k]['toState']});\n"
                    
            dispatchString += f"{TAB*5}break;\n\n"

        dispatchString += f"{TAB*4}default:\n{TAB*5}break;\n{TAB*3}}}\n{TAB*3}break;\n\n"
        
    dispatchString += "\t}\n}\n\n"
    
    return dispatchString
    

# Creates main() function definition - hardcoded, potentially allow for timing configuration of while loop 
# which is currently just a while(1) loop calling eventGenerator() and then dispatch(event) if an event occurs 
def createMainString():
    mainString = "int main()\n{\n\tEvent_t currentEvent = NONE;\n\twhile (1) {\n\t\tcurrentEvent = eventGenerator();\n"
    mainString += "\t\tif (currentEvent != NONE) {\n\t\t\tdispatch(currentEvent);\n\t\t}\n\t}\n\n\treturn 0;\n}"
    
    return mainString

def getJson(filename: str) -> str:
    with open(filename, mode="r") as json_file:
        data = json.load(json_file)

    return data

def convertJsonToCpp(data, userGeneratedDefinitionsFilename, outputCppSourceFilename):
    try:
        implementationIncludeString = createImplementationIncludeString(userGeneratedDefinitionsFilename)
    except:
        implementationIncludeString = createImplementationIncludeString()

    initString = createInitString(data['variables'], data['initialState'])
    tranString = createTranString()
    dispatchString = createDispatchString(data['states'], data['events'], data['transitions'])
    mainString = createMainString()
    
    definitionIncludeString = createDefinitionIncludeString()
    stateString = createStateString(data['states'])
    eventString = createEventString(data['events'])
    
    # Write all the strings to the source file sequentially
    with open("generatedDefinitions.h", "w") as definitions_file:
        definitions_file.write(definitionIncludeString)
        definitions_file.write(stateString)
        definitions_file.write(eventString)
        
    with open(outputCppSourceFilename, "w") as cpp_file:
        cpp_file.write(implementationIncludeString)
        cpp_file.write(initString)
        cpp_file.write(tranString)
        cpp_file.write(dispatchString)
        cpp_file.write(mainString)

    