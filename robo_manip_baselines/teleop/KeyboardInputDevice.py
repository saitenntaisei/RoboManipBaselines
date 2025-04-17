import numpy as np
import pinocchio as pin
import threading
from pynput import keyboard

from .InputDeviceBase import InputDeviceBase


class KeyboardInputDevice(InputDeviceBase):
    """Keyboard for teleoperation input device."""

    def __init__(
        self,
        arm_manager,
        pos_scale=1e-2,
        rpy_scale=5e-3,
        gripper_scale=5.0,
    ):
        super().__init__()

        self.arm_manager = arm_manager
        self.pos_scale = pos_scale
        self.rpy_scale = rpy_scale
        self.gripper_scale = gripper_scale
        
        # キー入力の状態を保持する辞書
        self.key_states = {
            # 位置制御キー
            'w': False,  # +Y方向
            's': False,  # -Y方向
            'a': False,  # -X方向
            'd': False,  # +X方向
            'q': False,  # +Z方向
            'e': False,  # -Z方向
            
            # 回転制御キー
            'i': False,  # +pitch
            'k': False,  # -pitch
            'j': False,  # +yaw
            'l': False,  # -yaw
            'u': False,  # +roll
            'o': False,  # -roll
            
            # グリッパー制御キー
            'z': False,  # グリッパーを開く
            'x': False,  # グリッパーを閉じる
        }
        
        self.listener = None
        self.listener_thread = None

    def connect(self):
        if self.connected:
            return

        # キーボードリスナーを作成
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release)
        
        # 別スレッドでキーボードリスナーを開始
        self.listener_thread = threading.Thread(target=self._start_listener)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        self.connected = True
        print("[KeyboardInputDevice] Connected. Use WASD for XY movement, QE for Z, IJKL for rotation, ZX for gripper.")

    def _start_listener(self):
        self.listener.start()
        self.listener.join()

    def _on_press(self, key):
        try:
            k = key.char.lower()
            if k in self.key_states:
                self.key_states[k] = True
        except AttributeError:
            pass

    def _on_release(self, key):
        try:
            k = key.char.lower()
            if k in self.key_states:
                self.key_states[k] = False
        except AttributeError:
            pass

    def read(self):
        if not self.connected:
            raise RuntimeError(f"[{self.__class__.__name__}] Device is not connected.")
        
        # キー状態は_on_press/_on_releaseメソッドで自動的に更新されるため、
        # ここでは特別な処理は必要ありません

    def set_command_data(self):
        # 位置変化量を計算
        delta_pos = np.zeros(3)
        
        
        # X軸
        if self.key_states['w']:
            delta_pos[0] += self.pos_scale
        if self.key_states['s']:
            delta_pos[0] -= self.pos_scale
        
        # Y軸
        if self.key_states['d']:
            delta_pos[1] += self.pos_scale
        if self.key_states['a']:
            delta_pos[1] -= self.pos_scale
            
      
            
        # Z軸
        if self.key_states['q']:
            delta_pos[2] += self.pos_scale
        if self.key_states['e']:
            delta_pos[2] -= self.pos_scale
        
        # 回転変化量を計算
        delta_rpy = np.zeros(3)
        
        # Roll
        if self.key_states['u']:
            delta_rpy[0] += self.rpy_scale
        if self.key_states['o']:
            delta_rpy[0] -= self.rpy_scale
            
        # Pitch
        if self.key_states['i']:
            delta_rpy[1] += self.rpy_scale
        if self.key_states['k']:
            delta_rpy[1] -= self.rpy_scale
            
        # Yaw
        if self.key_states['j']:
            delta_rpy[2] += self.rpy_scale
        if self.key_states['l']:
            delta_rpy[2] -= self.rpy_scale

        # アームの目標位置・姿勢を更新
        target_se3 = self.arm_manager.target_se3.copy()
        target_se3.translation += delta_pos
        target_se3.rotation = pin.rpy.rpyToMatrix(*delta_rpy) @ target_se3.rotation

        self.arm_manager.set_command_eef_pose(target_se3)

        # グリッパーコマンドの設定
        gripper_joint_pos = self.arm_manager.get_command_gripper_joint_pos().copy()

        if self.key_states['z'] and not self.key_states['x']:
            gripper_joint_pos += self.gripper_scale
        elif self.key_states['x'] and not self.key_states['z']:
            gripper_joint_pos -= self.gripper_scale

        self.arm_manager.set_command_gripper_joint_pos(gripper_joint_pos)

    def disconnect(self):
        if self.connected:
            if self.listener:
                self.listener.stop()
            self.connected = False
            print("[KeyboardInputDevice] Disconnected.")