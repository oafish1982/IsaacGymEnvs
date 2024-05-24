import sys
import time

from xarm.wrapper import XArmAPI


def int_to_2bytes_list(value):
    value_in_hex = hex(value)[2:].zfill(4)
    return [int(value_in_hex[0:2], base=16), int(value_in_hex[2:4], base=16)]


class ParalleGripperOpenRB150:
    def __init__(self, arm, torque_limit=150, baudrate=115200):
        self._arm = arm
        print('+++++++++++++')
        self._arm.set_mode(0)
        print('+++++++++++++')
        self._arm.set_state(0)
        # code = self._arm.set_tgpio_modbus_baudrate(1000000)
        # code = self._arm.set_tgpio_modbus_baudrate(baudrate)
        code = self._arm.set_tgpio_modbus_baudrate(115200)
        self._arm.set_tgpio_digital(1, 1)
        self._arm.set_tgpio_digital(0, 1)
        print('set_tgpio_modbus_baudrate, code={}'.format(code))
        self.close_in_res = -5
        self.open_in_res = 210
        # self.open_in_res = 400

    def open(self):
        data = [0x01, 0x06, 0x00, 0x80]
        data += int_to_2bytes_list(self.open_in_res)
        res = self._arm.getset_tgpio_modbus_data(data, time_out=1000)
        return res

    def close(self):
        data = [0x01, 0x06, 0x00, 0x80]
        data += int_to_2bytes_list(self.close_in_res)
        res = self._arm.getset_tgpio_modbus_data(data, time_out=1000)
        return res

    def move(self, pos):
        assert pos >= self.close_in_res
        assert pos <= self.open_in_res

        data = [0x01, 0x06, 0x00, 0x80] + int_to_2bytes_list(pos)
        # start = time.time()
        res = self._arm.getset_tgpio_modbus_data(data, time_out=1000)
        print("move: ", pos)
        return res

    def move_out_of_10(self, ratio):
        assert ratio <= 9
        pos = (self.open_in_res - self.close_in_res) * \
            ratio / 10 + self.close_in_res
        print(f'pos is {pos}')
        res = self.move(int(pos))
        return res

    def get_pos(self):
        data = [0x01, 0x03, 0x01, 0x01, 0x00, 0x02]
        res = self._arm.getset_tgpio_modbus_data(data, time_out=1000)
        return res

    def change_goal_current_val(self, current_val):
        data = [0x01, 0x06, 0x00, 0x81] + int_to_2bytes_list(current_val)
        res = self._arm.getset_tgpio_modbus_data(data, time_out=1000)
        return res


if __name__ == "__main__":
    action_list = "oc"
    ip = sys.argv[1]
    if len(sys.argv) >= 3:
        action_list = sys.argv[2]
    arm = XArmAPI(ip, is_radian=True)
    pga = ParalleGripperOpenRB150(arm)
    time.sleep(1)
    arm.clean_error()
    tmp = ""
    # tmp = pga.move_out_of_10(9)
    tmp = pga.move(int(0))
    print(tmp)
    time.sleep(0.5)

    # for action in action_list:
    #     print(action)
    #     arm.clean_error()
    #     res = ""
    #     if action == "o":
    #         res = pga.open()

    #     elif action == "c":
    #         res = pga.close()

    #     elif action == "s":
    #         time.sleep(1)

    #     elif action.isdecimal():
    #         val = int(action)
    #         res = pga.move_out_of_10(val)
    #     print(res)

    #     time.sleep(2)
