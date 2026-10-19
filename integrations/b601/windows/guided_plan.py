"""由操作者确认开始的分姿态采样计时；准备时间不计入样本。"""


class PoseCapture:
    def __init__(self, hold_s=10, poses=4):
        self.hold_s, self.poses = hold_s, poses
        self.index, self.seconds = 0, 0.0
        self.collecting = False
        self.last = self.stable_since = self.started = None
        self.intervals = []

    @property
    def complete(self):
        return self.index == self.poses

    def begin(self, now):
        if self.complete or self.collecting:
            return
        self.collecting = True
        self.started = self.last = now
        self.stable_since = None
        self.seconds = 0.0

    def update(self, now, steady):
        if not self.collecting:
            return False
        dt = max(0.0, now - self.last)
        self.last = now
        if not steady or dt > 0.3:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since >= 1:
            self.seconds += dt
        if self.seconds + 1e-7 >= self.hold_s:
            self.intervals.append([self.started, now])
            self.index += 1
            self.collecting = False
            self.seconds = 0.0
            self.stable_since = None
            return True
        return False
