#include <iostream>
#include "generatedDefinitions.h"
#include "user_definitions.h"

using namespace std;
using namespace generatedDefinitions;
using namespace user_definitions;

State_t currentState = STATE1;
Counters_t testCount = {0, 0};

void tran(State_t nextState) {
	currentState = nextState;
}

void dispatch(Event_t event) {
	switch(currentState) {
		case STATE1:
			switch(event) {
				 case EVENT1:
					testCount.transitionCounter++;
					printCounters(testCount);
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
					testCount.transitionCounter++;
					printCounters(testCount);
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
					if (testCount.cycleCounter < 3) {
						testCount.transitionCounter++;
						testCount.cycleCounter++;
						printCounters(testCount);
						cout << "Transitioned to STATE1\n";
						tran(STATE1);
					}
					else if (testCount.cycleCounter >= 3) {
						cout << "Transitioned to STATE3\n";
						printCounters(testCount);
						cout << "But all good things must come to an end\n";
						tran(STATE3);
					}
					break;

			}
			break;

	}
}

int main()
{
	Event_t currentEvent = NONE;
	while (1) {
		currentEvent = eventGenerator();
		if (currentEvent != NONE) {
			dispatch(currentEvent);
		}
	}

	return 0;
}