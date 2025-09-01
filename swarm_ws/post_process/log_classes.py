import json




class LogFile:
    def __init__(self) -> None:
        self.all_lines = []
        self.PositionLogs = PositionLogs()
        self.MissionLogs = MissionLogs()
        self.BatteryLogs = BatteryLogs()
        self.SetupLogs = SetupLogs()

    def parse_file(self, file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
            for line in lines:
                self.all_lines.append(line)
                log_line = LogLine(line)
                if log_line.log_class == 'position':
                    self.PositionLogs.log_lines.append(log_line)
                elif log_line.log_class == 'mission':
                    self.MissionLogs.log_lines.append(log_line)
                elif log_line.log_class == 'battery':
                    self.BatteryLogs.log_lines.append(log_line)
                elif log_line.log_class == 'setup':
                    self.SetupLogs.log_lines.append(log_line)

  
class LogLine:
    def __init__(self, line) -> None:
        self.line = line
        self.time = None
        self.raw_data = None
        self.type = None
        self.msg = None

        self.parse_line()

    def parse_line(self):
        line = self.line
        line.split(' ')
        self.time = line[0]
        self.raw_data = line[1:]

    def extract_data(self):
        split_data = self.raw_data.split(' ')
        self.type = split_data[0]
        self.msg = split_data[1]

        def __str__(self) -> str:
            return f'{self.time} {self.type} {self.msg}'

class Logs:
    def __init__(self):
        self.log_lines = []

class Coordinates():
    def __init__(self, x, y, z) -> None:
        self.x = x
        self.y = y
        self.z = z

class Positions():
    def __init__(self, entry, time) -> None:
        self.x = None
        self.y = None
        self.z = None
        self.coordinates = None
        self.time = time
        self.entry_json = entry
        self.data_json = entry['data']
        self.name = entry['name']
        self.extract_data()

    def extract_data(self):
        data = self.data_json
        self.coordinates = Coordinates(data['x'], data['y'], data['z'])


class MissionLogs(Logs):
    def __init__(self):
        super().__init__()
        self.log_class = 'mission'
        self.positions = {}
        self.failed_positions = []

    def extract_positions(self):
        for log_line in self.log_lines:
            msg = log_line.msg
            if msg.contains('ERROR!'):
                self.failed_positions.append(log_line)
                continue
            msg_dict = json.loads(msg)
            for entry in msg_dict:
                name = entry['name']
                position = Positions(entry, log_line.time)
                if name not in self.positions:
                    self.positions[name] = []
                self.positions[name].append(position)

            
class PositionLogs(Logs):
    def __init__(self):
        super().__init__()
        self.log_class = 'position'

class BatteryLogs(Logs):
    def __init__(self):
        super().__init__()
        self.log_class = 'battery'

class SetupLogs(Logs):
    def __init__(self):
        super().__init__()
        self.log_class = 'setup'
