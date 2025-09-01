import logging
from .process_logs import process_logs
from .log_classes import Coordinates
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)

LOG_DIR = 'placeholder'  # TODO: Replace with the actual log directory


class MissionErrors:
    def __init__(self, mission_logs) -> None:
        self.positions = mission_logs.positions
        self.goal_positions = self.positions['goal']
        self.current_positions = self.positions['current']
        self.failed_positions = mission_logs.failed_positions
        self.times = {}
        self.normalised_times = {}
        self.calculate_errors()

    def calculate_errors(self):
        for goal, current in zip(self.goal_positions, self.current_positions):
            if goal.time != current.time:
                logger.error('Goal and current positions have different timestamps')
                return

            error = Coordinates(goal.x - current.x, goal.y - current.y, goal.z - current.z)
            self.times[goal.time] = error

        start_time = self.goal_positions[0].time

        for time in self.times.keys():
            self.normalised_times[time - start_time] = self.times[time]


class Mission:
    def __init__(self, mission_logs) -> None:
        self.mission_logs = mission_logs
        self.errors = None

    def process_mission(self):
        self.errors = MissionErrors(self.mission_logs)
        self.plot_mission()

    def plot_mission(self, save_path=None):
        fig, ax = plt.subplots()

        times = sorted(self.errors.normalised_times.keys())
        errors_x = [self.errors.normalised_times[time].x for time in times]
        errors_y = [self.errors.normalised_times[time].y for time in times]
        errors_z = [self.errors.normalised_times[time].z for time in times]

        ax.plot(times, errors_x, 'r-', label='x')
        ax.plot(times, errors_y, 'g-', label='y')
        ax.plot(times, errors_z, 'b-', label='z')

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error (m)')
        ax.set_title('Mission Error')
        ax.grid(True)
        ax.legend()

        if save_path is not None:
            plt.savefig(save_path)
        plt.show()


def main():
    logs = process_logs(LOG_DIR)
    if logs is None:
        logger.error('No logs found')
        return

    mission_logs = logs.get_mission_logs()
    if mission_logs is None:
        logger.error('No mission logs found')
        return

    mission = Mission(mission_logs)
    mission.process_mission()


if __name__ == '__main__':
    main()
