.PHONY: all clean ignore

all:
	colcon build

clean:
	colcon build --clean

ignore:
# This will filter out the target name "ignore" from MAKECMDGOALS,
# leaving only the extra arguments.
	colcon build --packages-ignore $(filter-out $@,$(MAKECMDGOALS))

# Prevent make from treating the extra arguments as separate targets.
%:
	@:

select:
	colcon build --packages-select $(filter-out $@,$(MAKECMDGOALS))


sim:
	export SIM_BUILD=1 && colcon build

input:
	colcon build --packages-select controller_msgs user_input