import ast
import os
from pathlib import Path

from dotenv import load_dotenv

# Load values from .env file
load_dotenv()

cfp = os.path.abspath(__file__)
cwd = Path(os.path.dirname(cfp))

# Access environment variables from the .env file
NUM_BATCHES = int(os.getenv('NUM_BATCHES', "64"))
NUM_STEPS = int(os.getenv('NUM_STEPS', "128"))
EPOCH = int(os.getenv('EPOCH', "2"))
NUM_OBS = int(os.getenv('NUM_OBS', "454"))  # FIXED: Match training (was 226)
ACT_SIZE = int(os.getenv('ACT_SIZE', "2"))  # FIXED: 2 actions (linear, angular)
LEARNING_RATE = float(os.getenv('LEARNING_RATE', "7e-5"))  # Match training
COEFF_ENTROPY = float(os.getenv('COEFF_ENTROPY', "1e-3"))  # Match training
COEFF_VALUE_LOSS = float(os.getenv('COEFF_VALUE_LOSS', "1.0"))  # Match training
CLIP_VALUE = float(os.getenv('CLIP_VALUE', "0.1"))
GAMMA = float(os.getenv('GAMMA', "0.99"))
LAMBDA = float(os.getenv('LAMBDA', "0.94"))  # Match training (was 0.95)
LASER_NUM_FRAMES = float(os.getenv('LASER_NUM_FRAMES', "3"))
LASER_RANGE = float(os.getenv('LASER_RANGE', "6.0"))  # Match training max range
BATCH_SIZE = int(os.getenv('BATCH_SIZE', "1024"))  # Match training
LASER_HIST = int(os.getenv('LASER_HIST', "3"))

ROBOT_RADIUS = float(os.getenv('ROBOT_RADIUS', "0.13"))
ROBOT_WHEEL_DISTANCE = float(os.getenv('ROBOT_WHEEL_DISTANCE', "0.21"))
SAFE_DISTANCE = float(os.getenv('SAFE_DISTANCE', "0.13"))
NUM_ROBOTS = int(os.getenv('NUM_ROBOTS', "1"))  # FIXED: Single robot deployment

# CRITICAL FIX: Match training bounds to prevent robot from going too fast!
# Training: [[0, -1.0], [0.7, 1.0]] means max linear vel = 0.7 m/s, max angular = 1.0 rad/s
ACTION_BOUND = ast.literal_eval(os.getenv('ACTION_BOUND', "[[0, -1.0], [0.3, 1.0]]"))
ROBOT_PORT = os.getenv("ROBOT_PORT", "/dev/ttyUSB0")
BAUDRATE = int(os.getenv("BAUDRATE", "115200"))

START_BYTE = 0xAB
END_BYTE = 0x03

CMD_VEL = 0x01
CMD_VEL_LEFT = 0x02
CMD_VEL_RIGHT = 0x03
GET_SPEED = 0x04
TUNE_PID_LEFT = 0x05
TUNE_PID_RIGHT = 0x06
GET_PID_DATA = 0x07

LEFT_WHEEL = 0
RIGHT_WHEEL = 1

# Path
POLICY_PATH = cwd / Path('policy')
MAP_FILE_PATH = cwd / Path('map')
