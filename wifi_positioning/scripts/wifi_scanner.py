import subprocess
import re

class WiFiScanner:
    def __init__(self):
        self.target_essids = {
            "0_LP": 0,
            "Yawo": 1,
            "NUMERICABLE-8378": 2,
            "turtlebot": 2,
            "Amirsharata": 3,
            "Amirsharata01": 4,
        }

    def scan_wifi(self):
        result = subprocess.run(['sudo', 'iwlist', 'wlan0', 'scan'], capture_output=True, text=True)
        return result.stdout

    def parse_scan_results(self, scan_output):
        lines = scan_output.split('\n')
        essid = None
        signal = None
        address = None
        signals = [None, None, None, None, None]

        for line in lines:
            line = line.strip()
            if line.startswith("Cell"):
                if essid and signal and address:
                    if essid in self.target_essids:
                        index = self.target_essids[essid]
                        signals[index] = signal
                address = re.search(r"Address: (\S+)", line).group(1)
                essid = None
                signal = None
            elif "ESSID" in line:
                essid = re.search(r'ESSID:"(.*?)"', line).group(1)
            elif "Signal level" in line:
                signal_match = re.search(r'Signal level=(-?\d+) dBm', line)
                if signal_match:
                    signal = signal_match.group(1)

        if essid and signal and address:
            if essid in self.target_essids:
                index = self.target_essids[essid]
                signals[index] = signal

        signals = [signal for signal in signals if signal is not None]
        return signals

    def scan_and_get_data(self):
        scan_output = self.scan_wifi()
        return self.parse_scan_results(scan_output)

    def average(self, values):
        # Calculer la moyenne en évitant les valeurs 'N/A'
        valid_values = [int(v) for v in values if v != 'N/A']
        if not valid_values:
            return 'N/A'
        print(valid_values)
        return sum(valid_values) / len(valid_values)
