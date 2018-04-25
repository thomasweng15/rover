import json

class Config:
    def __init__(self, fname="/home/pi/catkin_ws/src/rover/scripts/config.json"):
        self.config = None
        with open(fname, 'r') as f:
            self.config = json.load(f)

    def get(self, section, pin_name):
        try: 
            return self.config[section][pin_name]
        except KeyError:
            return None 

if __name__ == '__main__':
    c = Config()
    print c.get("motors", "in1")
