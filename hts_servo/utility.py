import enum

class Command(enum.IntEnum):
    SERVO_MOVE_TIME_WRITE = 1 # length: 7
    SERVO_MOVE_TIME_READ = 2 # length: 3
    SERVO_MOVE_TIME_WAIT_WRITE = 7 # length: 7
    SERVO_MOVE_TIME_WAIT_READ = 8 # length: 3
    SERVO_MOVE_START = 11 # length: 3
    SERVO_MOVE_STOP = 12 # length: 3
    SERVO_ID_WRITE = 13 # length: 4
    SERVO_ID_READ = 14 # length: 3
    SERVO_ANGLE_OFFSET_ADJUST = 17 # length: 4
    SERVO_ANGLE_OFFSET_WRITE = 18 # length: 3
    SERVO_ANGLE_OFFSET_READ = 19 # length: 3
    SERVO_ANGLE_LIMIT_WRITE = 20 # length: 7
    SERVO_ANGLE_LIMIT_READ = 21 # length: 3
    SERVO_VIN_LIMIT_WRITE = 22 # length: 7
    SERVO_VIN_LIMIT_READ = 23 # length: 3
    SERVO_TEMP_MAX_LIMIT_WRITE = 24 # length: 4
    SERVO_TEMP_MAX_LIMIT_READ = 25 # length: 3
    SERVO_TEMP_READ = 26 # length: 3
    SERVO_VIN_READ = 27 # length: 3
    SERVO_POS_READ = 28 # length: 3
    SERVO_OR_MOTOR_MODE_WRITE = 29 # length: 7
    SERVO_OR_MOTOR_MODE_READ = 30 # length: 3
    SERVO_LOAD_OR_UNLOAD_WRITE = 31 # length: 4
    SERVO_LOAD_OR_UNLOAD_READ = 32 # length: 3
    SERVO_LED_CTRL_WRITE = 33 # length: 4
    SERVO_LED_CTRL_READ = 34 # length: 3
    SERVO_LED_ERROR_WRITE = 35 # length: 4
    SERVO_LED_ERROR_READ = 36 # length: 3

class ServoObject(int):
    """
    舵机对象
    继承自 int 代表 ID
    """
    pass

SERVO_BRODCAST = ServoObject(0xfe)  # 广播ID

def param2bytes(param: int, len: int = 2, is_signed: bool = False) -> bytearray:
    """
    将参数转换为小端头的字节串
    [Low 8 bits, High 8 bits]
    """
    return param.to_bytes(len, "little", is_signed)

def bytes2param(byte_arr: bytearray | bytes, is_signed: bool = False) -> int:
    """
    将小端头的字节串转换为参数
    """
    return int.from_bytes(byte_arr, "little", signed = is_signed)

def degree2cmd(degree: float) -> int:
    """
    将角度转换为指令格式
    """
    if (degree < 0) or (degree > 240):
        raise ValueError("Degree should be in range [0, 240]")
    return int(degree * 1000 / 240)

def degree2offset(degree: float) -> int:
    """
    将角度转换为偏移值格式
    """
    if (degree < -30) or (degree > 30):
        raise ValueError("Degree should be in range [-30, 30]")
    return int(degree * 125 / 30)
