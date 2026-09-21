"""馬達驅動層。

bridge_node 只透過 base.MotorDriver 的介面跟驅動板講話，
換板子時只要在這個目錄下新增一個實作，bridge_node 完全不用改。
"""

from .base import MotorDriver, DriverError          # noqa: F401
from .loopback import LoopbackDriver                # noqa: F401

# driver_type 參數 -> 類別。接上真實驅動板時在這裡多一行。
DRIVERS = {
    'loopback': LoopbackDriver,
}


def create_driver(driver_type, **kwargs):
    """依 driver_type 建立驅動實例，未知型別直接報錯（不要 fallback 成假驅動，
    那會讓實車上「驅動沒接好」變成「車子看起來正常但不會動」）。"""
    if driver_type not in DRIVERS:
        raise DriverError(
            "未知的 driver_type '%s'，目前支援: %s"
            % (driver_type, ', '.join(sorted(DRIVERS))))
    return DRIVERS[driver_type](**kwargs)
