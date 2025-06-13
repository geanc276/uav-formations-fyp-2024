#include <iostream>
using namespace std;

typedef enum State {
	STATE1,
	STATE2,
	STATE3
} State_t;

typedef enum Event {
	NONE,
	EVENT1,
	EVENT2,
	EVENT3
} Event_t;

State_t currentState;
int transitionCounter;
int cycleCounter;

void init() {
	currentState = STATE1;
	transitionCounter = 0;
	cycleCounter = 0;
}

void tran(State_t nextState) {
	currentState = nextState;
}

void dispatch(Event_t event) {
	switch(currentState) {
		case STATE1:
			switch(event) {
				 case EVENT1:
					transitionCounter++;
					cout << "Transitioned to STATE2\n";
					tran(STATE2);
					break;

				 case EVENT2:
					break;

				 case EVENT3:
					break;

			}
			break;

		case STATE2:
			switch(event) {
				 case EVENT1:
					break;

				 case EVENT2:
					transitionCounter++;
					cout << "Transitioned to STATE3\n";
					tran(STATE3);
					break;

				 case EVENT3:
					break;

			}
			break;

		case STATE3:
			switch(event) {
				 case EVENT1:
					break;

				 case EVENT2:
					break;

				 case EVENT3:
					if (cycleCounter < 3) {
						transitionCounter++;
						cycleCounter++;
						cout << "Transitioned to STATE1\n";
						cout << "Transition count is " << transitionCounter << "\n";
						tran(STATE1);
					} else if (cycleCounter >= 3) {
						cout << "Cycle count is " << cycleCounter << "\n" << "But all good things must come to an end\n";
						tran(STATE3);
					}
					break;

			}
			break;

	}
}

Event_t eventGenerator() {
	char input;
	cout << "Input a character: ";
	cin >> input;
	switch(input) {
		case 'a':
			cout << "EVENT1 triggered\n";
			return EVENT1;
		
		case 'b':
			cout << "EVENT2 triggered\n";
			return EVENT2;
			
		case 'c':
			cout << "EVENT3 triggered\n";
			return EVENT3;
	}
	return NONE;
}

int main()
{
	init();
	Event_t currentEvent = NONE;
	while (1) {
		currentEvent = eventGenerator();
		if (currentEvent != NONE) {
			dispatch(currentEvent);
		}
	}
	
	return 0;
}



