import math
import time
from sim2real.robot_controller import RobotContorller
from xarm.wrapper import XArmAPI
from sim2real.control_openRB150_with_modbus_rtu import ParalleGripperOpenRB150


class lite6Follower:
    
    def __init__(self):
        self.use_gripper = True
        self.use_follower = True
        self.gripper_angle_offset_max = 33
        self.gripper_scale = 230/33  #手指的33mm行程对应夹爪齿轮的230度
        self.ip_robot = "192.168.1.206"
        try:
            self.arm = XArmAPI(self.ip_robot, is_radian=True)
            self.arm.clean_error()
            self.arm.set_mode(1)
            self.arm.set_state(0)
            DH_params = None
            self.rc = RobotContorller(self.arm, DH_params, filter_size=5, filter_type=None)
            self.pga = ParalleGripperOpenRB150(self.arm)  
        except:
            print("连接失败")
    def run_lite6(self, angles):
        # issacgym中获取的手指完全打开的行程是0(单位m)，闭合过程中行程增大，两个手指最大间距66mm，每个手指最多走33mm，gripper从闭合到完全打开是0到230度
        gripper_pos = int((self.gripper_angle_offset_max - angles[6]*1000) * self.gripper_scale)
        gripper_pos = max(min(gripper_pos, 240), 0)
        print("角度:",[round(angle * 180 / math.pi, 1) for angle in angles], gripper_pos)
        # print("弧度:",[angle for angle in angles])
        # self.arm.set_mode(mode=1)
        if self.use_follower:
            self.arm.clean_error()
            self.arm.set_mode(1)
            self.arm.set_state(0)
            self.rc.move_robot_joint(angles[:6], is_radian=True)
        if self.use_gripper:
            self.pga.move(gripper_pos)
        time.sleep(0.01)
