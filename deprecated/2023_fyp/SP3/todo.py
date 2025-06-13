import PySimpleGUI as sg

states = []
events = []
variables = []

stateLayout = [
    [sg.Text('States')],
    [sg.InputText('Enter state name', key='state_item'), sg.Button('Add')],
    [sg.Listbox(values=states, size=(40, 10), key="items"), sg.Button('Delete')],  
    [sg.Button('Next')],
]

eventLayout = [
    [sg.Text('Events')],
    [sg.InputText('Enter event name', key='event_item'), sg.Button('Add')],
    [sg.Listbox(values=events, size=(40, 10), key="items"), sg.Button('Delete')],
    [sg.Button('Next')],
]

variableLayout = [
    [sg.Text('Variables')],
    [sg.InputText('Enter variable declaration', key='variable_item'), sg.Button('Add')],
    [sg.Listbox(values=variables, size=(40, 10), key="items"), sg.Button('Delete')],
    [sg.Button('Next')],
]

transitionLayout = [
    
]

filenameLayout = [
    
]

layouts = [stateLayout, eventLayout, variableLayout, transitionLayout, filenameLayout]

currentWindowState = 0

window = sg.Window('State App', layouts[currentWindowState])

while True:  # Event Loop
    event, values = window.Read()
    if (sg.WIN_CLOSED, 'Exit'):
        break
            
    if event == "Next" and currentWindowState < 2:
        currentWindowState += 1
        window = sg.Window('State App', layouts[currentWindowState])
    elif event == None:
        break
    elif currentWindowState == 0:
        if event == "Add":
            states.append(values['state_item'])
            window['items'].Update(values=states)
            window['Add'].Update("Add")
        elif event == "Delete":
            states.remove(values["items"][0])
            window['items'].Update(values=states)
    elif currentWindowState == 1:
        if event == "Add":
            events.append(values['event_item'])
            window['items'].Update(values=events)
            window['Add'].Update("Add")
    elif currentWindowState == 2:
        if event == "Add":
            variables.append(values['variable_item'])
            window['items'].Update(values=variables)
            window['Add'].Update("Add")
with open("log.txt", "w") as log_file:
    for state in states:
        log_file.write(f"{state}\n")
    for event in events:
        log_file.write(f"{event}\n")
    for variable in variables:
        log_file.write(f"{variables}\n")
window.close()
      