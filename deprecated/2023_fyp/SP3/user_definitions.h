#pragma once
#include <iostream>
#include "generatedDefinitions.h"

using namespace std;
using namespace generatedDefinitions;

namespace user_definitions {
	typedef struct Counters {
		int transitionCounter;
		int cycleCounter;
	} Counters_t;
	
	void printCounters(Counters_t counters) {
		cout << "Transition counter: " << counters.transitionCounter << "\n";
		cout << "Cycle counter: " << counters.cycleCounter << "\n";
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
}
