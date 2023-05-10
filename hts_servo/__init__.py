import enum
import serial
from typing import Optional, Tuple

from utility import *

# TODO: 记得使用的时候异常处理
class ServoController:
    """
    舵机控制器
    单例
    """
    handle: Optional[serial.Serial] = None

    def __init__(self) -> None:
        pass

    @classmethod
    def check_handle(cls):
        """检查是否连接"""
        if cls.handle is None:
            raise RuntimeError("Servo controller is not connected")

    @classmethod
    def connect(cls, device: str, baudrate: Optional[int] = 115200, timeout: Optional[float] = 1.0) -> None:
        """连接舵机"""
        if cls.handle is not None:
            print("Servo controller is already connected")
        else:
            cls.handle = serial.Serial(device, baudrate, timeout=timeout)
            print("Servo controller connected")

    @classmethod
    def disconnect(cls) -> None:
        """断开连接"""
        cls.check_handle()
        cls.handle.close()
        cls.handle = None
        print("Servo controller disconnected")

    @classmethod
    def send_command(cls, servo_id: ServoObject, command: Command, params: bytearray = bytearray()) -> None:
        """发送指令"""
        cls.check_handle()
        # 指令头
        buffer = bytearray(b'\x55\x55')
        # 指定舵机 ID
        buffer += servo_id.to_bytes(1, "little")
        # 指令长度
        buffer += (len(params) + 3).to_bytes(1, "little")
        # 指令
        buffer += command.to_bytes(1, "little")
        # 参数
        buffer += params
        # 校验和
        buffer += (~sum(buffer[2:]) % 256).to_bytes(1, "little")
        # 发送
        try:
            cls.handle.write(buffer)
            # print(f"Write command: {command.name}\n"
            #       f"\tBuffer: {buffer}")
        except Exception as e:
            print(f"Error: {e}\n"
                  f"\tHappened during sending command: {command.name}\n"
                  f"\tBuffer: {buffer}")
            raise e

    @classmethod
    def read_response(cls, servo_id: ServoObject, command: Optional[Command] = None) -> Tuple[Command, bytearray]:
        """读取响应"""
        cls.check_handle()
        # 等待响应
        while cls.handle.in_waiting == 0:
            pass
        # 读取响应头
        buffer = cls.handle.read(4)
        # 校验响应头
        if buffer[:2] != b'\x55\x55':
            raise RuntimeError(f"Invalid response header: {hex(buffer[:2])}")
        if servo_id != SERVO_BRODCAST and buffer[2] != servo_id:
            raise RuntimeError(f"Invalid response servo ID ({servo_id}): {buffer[2]}")
        # 读取响应长度
        length = buffer[3]
        # 读取响应内容
        buffer += cls.handle.read(length - 1)
        if command is not None and buffer[4] != command:
            raise RuntimeError(f"Invalid response command ({command.name}: {hex(command)}): {hex(buffer[4])}")
        # 校验响应内容
        expected_checksum = ~sum(buffer[2:-1]) % 256
        if buffer[-1] != expected_checksum:
            raise RuntimeError(f"Invalid response checksum ({hex(expected_checksum)}): {hex(buffer[-1])}")
        # 返回响应
        return Command(buffer[4]), buffer[5:-1]

    @classmethod
    def set_move_time(cls, position: int, time: int, is_wait: bool = False, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """
        在一定时间旋转至某角度
        is_wait == True 的时候会等待 start_move
        """
        cls.check_handle()
        if position < 0 or position > 1000:
            raise RuntimeError(f"Invalid position (0~1000): {position}")
        if time < 0 or time > 30000:
            raise RuntimeError(f"Invalid time (0~30000): {time}")
        if is_wait:
            cls.send_command(servo_id, Command.SERVO_MOVE_TIME_WAIT_WRITE, param2bytes(position, 2) + param2bytes(time))
        else:
            cls.send_command(servo_id, Command.SERVO_MOVE_TIME_WRITE, param2bytes(position) + param2bytes(time))

    @classmethod
    def get_move_time(cls, servo_id: ServoObject, is_wait: bool = False) -> Tuple[int, int]:
        """
        获取 set_move_time 设置的内容
        return (角度, 时间)
        """
        cls.check_handle()
        if is_wait:
            cls.send_command(servo_id, Command.SERVO_MOVE_TIME_WAIT_READ)
            response = cls.read_response(servo_id, Command.SERVO_MOVE_TIME_WAIT_READ)
        else:
            cls.send_command(servo_id, Command.SERVO_MOVE_TIME_READ)
            response = cls.read_response(servo_id, Command.SERVO_MOVE_TIME_READ)[1]
        return bytes2param(response[0:2]), bytes2param(response[2:4])

    @classmethod
    def start_move(cls, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """开始移动"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_MOVE_START)

    @classmethod
    def stop_move(cls, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """停止移动"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_MOVE_STOP)

    @classmethod
    def set_id(cls, target_id: int, servo_id: ServoObject = SERVO_BRODCAST):
        """
        设置 ID
        (0~253)
        """
        cls.check_handle()
        if id < 0 or id > 253:
            raise RuntimeError()
        if servo_id == SERVO_BRODCAST:
            import warnings
            warnings.warn("You are broadcasting writing id!", RuntimeWarning)
        cls.send_command(servo_id, Command.SERVO_MOVE_STOP, param2bytes(target_id, 1))
    
    @classmethod
    def get_id(cls, servo_id: ServoObject = SERVO_BRODCAST) -> ServoObject:
        """获取 ID"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_ID_READ)
        response = cls.read_response(servo_id, Command.SERVO_ID_READ)[1]
        return ServoObject(response[0])
    
    @classmethod
    def set_angle_offset(cls, offset: int, is_forever: bool = False, servo_id: ServoObject = SERVO_BRODCAST):
        """
        设置角度偏置
        可设置是否掉电保留 is_forever
        """
        cls.check_handle()
        if offset < -125 or offset > 125:
            raise ValueError(f"Invalid offset (-125~125): {offset}")
        if is_forever:
            cls.send_command(servo_id, Command.SERVO_ANGLE_OFFSET_WRITE, param2bytes(offset, 1, True))
        else:
            cls.send_command(servo_id, Command.SERVO_ANGLE_OFFSET_ADJUST, param2bytes(offset, 1, True))

    @classmethod
    def get_angle_offset(cls, servo_id: ServoObject = SERVO_BRODCAST) -> int:
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_ID_READ)
        response = cls.read_response(servo_id, Command.SERVO_ID_READ)[1]
        return bytes2param(response, True)
    
    @classmethod
    def set_angle_limit(cls, min: int, max: int, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """设置角度限制"""
        cls.check_handle()
        if min < 0 or min > 1000:
            raise RuntimeError(f"Invalid min angle (0~1000): {min}")
        if max < 0 or max > 1000:
            raise RuntimeError(f"Invalid max angle (0~1000): {max}")
        cls.send_command(servo_id, Command.SERVO_ANGLE_LIMIT_WRITE, param2bytes(min) + param2bytes(max))

    @classmethod
    def get_angle_limit(cls, servo_id: ServoObject) -> Tuple[int, int]:
        """
        读取角度限制
        return (最小角度, 最大角度)
        """
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_ANGLE_LIMIT_READ)
        response = cls.read_response(servo_id, Command.SERVO_ANGLE_LIMIT_READ)[1]
        return bytes2param(response[0:2]), bytes2param(response[2:4])

    @classmethod
    def set_vin_limit(cls, min: int, max: int, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """设置输入电压限制"""
        cls.check_handle()
        if min < 4500 or min > 12000:
            raise RuntimeError(f"Invalid min vin (4500~12000): {min}")
        if max < 4500 or max > 12000:
            raise RuntimeError(f"Invalid max vin (4500~12000): {max}")
        cls.send_command(servo_id, Command.SERVO_VIN_LIMIT_WRITE, param2bytes(min) + param2bytes(max))

    @classmethod
    def get_vin_limit(cls, servo_id: ServoObject) -> Tuple[int, int]:
        """
        读取输入电压限制
        return (最小角度, 最大角度)
        """
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_VIN_LIMIT_READ)
        response = cls.read_response(servo_id, Command.SERVO_VIN_LIMIT_READ)[1]
        return bytes2param(response[0:2]), bytes2param(response[2:4])

    @classmethod
    def set_max_temp_limit(cls, max: int, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """设置最高温度限制"""
        cls.check_handle()
        if max < 50 or max > 100:
            raise RuntimeError(f"Invalid max temp (50~100): {max}")
        cls.send_command(servo_id, Command.SERVO_TEMP_MAX_LIMIT_WRITE, param2bytes(max))

    @classmethod
    def get_max_temp_limit(cls, servo_id: ServoObject) -> int:
        """读取最高温度限制"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_TEMP_MAX_LIMIT_READ)
        response = cls.read_response(servo_id, Command.SERVO_TEMP_MAX_LIMIT_READ)[1]
        return bytes2param(response)

    @classmethod
    def get_temp(cls, servo_id: ServoObject) -> int:
        """读取温度"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_TEMP_READ)
        response = cls.read_response(servo_id, Command.SERVO_TEMP_READ)[1]
        return bytes2param(response)

    @classmethod
    def get_vin(cls, servo_id: ServoObject) -> int:
        """读取输入电压"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_VIN_READ)
        response = cls.read_response(servo_id, Command.SERVO_VIN_READ)[1]
        return bytes2param(response)

    @classmethod
    def get_pos(cls, servo_id: ServoObject) -> int:
        """读取角度位置"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_POS_READ)
        response = cls.read_response(servo_id, Command.SERVO_POS_READ)[1]
        return bytes2param(response)

    @classmethod
    def set_motor_mode(cls, speed: Optional[int], servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """
        设置为步进电机模式
        speed = None 切换为舵机模式
        """
        cls.check_handle()
        if speed is None:
            cls.send_command(servo_id, Command.SERVO_OR_MOTOR_MODE_WRITE, bytearray([0x00, 0x00, 0x00, 0x00]))
        else:
            if speed < -1000 or speed > 1000:
                raise ValueError(f"Invalid speed (-1000~1000): {speed}")
            cls.send_command(servo_id, Command.SERVO_TEMP_MAX_LIMIT_WRITE, bytearray([0x01, 0x00]) + param2bytes(speed, is_signed=True))

    @classmethod
    def get_motor_mode(cls, servo_id: ServoObject) -> Tuple[bool, int]:
        """
        读取步进电机模式
        (是否在步进电机模式, 速度)
        """
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_OR_MOTOR_MODE_READ)
        response = cls.read_response(servo_id, Command.SERVO_OR_MOTOR_MODE_READ)[1]
        return bool(response[0]), bytes2param(response[2:3])

    @classmethod
    def set_is_load(cls, is_load: bool, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """
        设置电机是否装载
        true: 装载上电, false: 卸载掉电
        """
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_LOAD_OR_UNLOAD_WRITE, bytearray([0x01 if is_load else 0x00]))

    @classmethod
    def get_is_load(cls, servo_id: ServoObject) -> bool:
        """读取电机装载状态"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_LOAD_OR_UNLOAD_READ)
        response = cls.read_response(servo_id, Command.SERVO_LOAD_OR_UNLOAD_READ)[1]
        return bool(response[0])
    
    @classmethod
    def set_led(cls, is_on: bool, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """设置 LED 状态"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_LED_CTRL_WRITE, bytearray([0x01 if is_on else 0x00]))

    @classmethod
    def get_led(cls, servo_id: ServoObject) -> bool:
        """读取 LED 状态"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_LED_CTRL_READ)
        response = cls.read_response(servo_id, Command.SERVO_LED_CTRL_READ)[1]
        return bool(response[0])
    
    @classmethod
    def set_led_error(cls, temp_warn: bool, vin_warn: bool, stick_warn: bool, servo_id: ServoObject = SERVO_BRODCAST) -> None:
        """
        设置 LED 警报启用
        temp_warn: 温度警报
        vin_warn: 电压警报
        stick_warn: 堵转警报
        """
        cls.check_handle()
        status += 1 if temp_warn else 0
        status += 2 if vin_warn else 0
        status += 4 if stick_warn else 0
        cls.send_command(servo_id, Command.SERVO_LED_ERROR_WRITE, bytearray([status]))

    @classmethod
    def get_led_error(cls, servo_id: ServoObject) -> Tuple[bool, bool, bool]:
        """读取 LED 警报启用设置"""
        cls.check_handle()
        cls.send_command(servo_id, Command.SERVO_LED_ERROR_READ)
        response = cls.read_response(servo_id, Command.SERVO_LED_ERROR_READ)[1]
        status = response[0]
        return bool(status & 0x01), bool(status & 0x02), bool(status & 0x04)


if __name__ == "__main__":
    import time
    ServoController.connect("/dev/ttyUSB0")
    id = ServoController.get_id()
    print(f"id: {id}")
    print(f"angle_offset: {ServoController.get_angle_offset(id)}")
    print(f"angle_limit: {ServoController.get_angle_limit(id)}")
    print(f"temp: {ServoController.get_temp(id)}")
    print(f"temp_max_limit: {ServoController.get_max_temp_limit(id)}")
    print(f"vin: {ServoController.get_vin(id)}")
    print(f"vin_limit: {ServoController.get_vin_limit(id)}")
    print(f"pos: {ServoController.get_pos(id)}")
    print(f"mode: {ServoController.get_motor_mode(id)}")
    print(f"is_load: {ServoController.get_is_load(id)}")
    print(f"led: {ServoController.get_led(id)}")
    print(f"led_error: {ServoController.get_led_error(id)}")

    ServoController.set_move_time(degree2cmd(0), 1000)
    print("Move to 0 degree")
    for i in range(5):
        print(f"Now angle: {ServoController.get_pos(id)}")
        time.sleep(0.2)
    mvt = ServoController.get_move_time(id)
    print(f"Move time: {mvt[0]}, {mvt[1]}")
    ServoController.set_move_time(degree2cmd(90), 1000)
    print("Move to 90 degree")
    for i in range(5):
        print(f"Now angle: {ServoController.get_pos(id)}")
        time.sleep(0.2)
    mvt = ServoController.get_move_time(id)
    print(f"Move time: {mvt[0]}, {mvt[1]}")

