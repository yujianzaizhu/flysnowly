"""导航进程仿真的独立参数，不同步到内部仿真的 config/world。"""


class Settings(object):
    def __init__(self):
        self.arc_speed = 0.40
        self.back_speed = 0.30
        self.turn_speed = 1.00
        self.arc_radius = 1.00


settings = Settings()

_RANGES = {
    "arc_speed": (0.05, 0.60),
    "back_speed": (0.05, 0.60),
    "turn_speed": (0.20, 1.80),
    "arc_radius": (0.50, 6.00),
}


def configure(**kwargs):
    for key, value in kwargs.items():
        if key not in _RANGES:
            raise ValueError("未知导航配置项: {}".format(key))
        low, high = _RANGES[key]
        try:
            value = float(value)
        except (TypeError, ValueError):
            raise ValueError("参数 {} 必须是数字".format(key))
        if not low <= value <= high:
            raise ValueError("参数 {} 超出范围 {}-{}".format(key, low, high))
        setattr(settings, key, value)


def reset():
    global settings
    settings = Settings()
