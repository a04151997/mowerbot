#!/usr/bin/env python3
"""馬達驅動的抽象介面。

【為什麼要有這一層】
接上真實驅動板時，只需要在 drivers/ 下新增一個繼承 MotorDriver 的類別，
把下面這幾個方法實作出來，bridge_node.py 一行都不用改。
反過來說，這個介面以外的東西（通訊協定、封包格式、暫存器位址）
都不應該洩漏到 bridge_node 裡。

【現在還不知道驅動板型號，所以這裡不做任何協定假設】
需要從板子文件查到什麼才能寫出實作，列在 drivers/README.md。
"""

import abc


class DriverError(Exception):
    """驅動層的錯誤（連不上、逾時、回傳格式不對等）"""


class MotorDriver(abc.ABC):
    """所有馬達驅動的共同介面。

    單位約定（整個介面只有這一種單位，不要在實作裡改）：
      角速度  rad/s，正值代表該側輪子「使車輛前進」的方向
      編碼器  ticks，累計值（不是增量）

    方向接反是硬體接線問題，用 bridge_node 的 invert_left / invert_right
    參數處理，不要在驅動實作裡偷偷改號 —— 否則之後很難查出是哪一層反的。
    """

    @abc.abstractmethod
    def connect(self):
        """建立與驅動板的連線。失敗要丟 DriverError，不要安靜地回傳。"""

    @abc.abstractmethod
    def disconnect(self):
        """關閉連線。要能重複呼叫而不出錯（節點關閉時可能會被叫到兩次）。"""

    @abc.abstractmethod
    def set_wheel_velocities(self, left_rad_s, right_rad_s):
        """設定左右輪的目標角速度 (rad/s)。

        四輪 skid-steer 的同側兩輪是一起驅動的，所以這裡只有左右兩個值。
        如果板子是四個獨立通道，在實作裡把同一個值送給同側的兩個通道。
        """

    @abc.abstractmethod
    def read_encoders(self):
        """回傳 (left_ticks, right_ticks) 累計值，讀取失敗時回傳 None。

        【失敗時一定要回傳 None，不要回傳上一次的值】
        回傳舊值的話里程計會以為車子停著，但車子其實在動，
        SLAM 會拿到與現實不符的位姿，那比「這一拍沒有資料」危險得多。
        """

    @abc.abstractmethod
    def stop(self):
        """立刻停止兩側馬達。這是安全路徑，實作要盡量簡單、不要有重試迴圈。"""

    def read_status(self):
        """回傳電壓、電流、溫度等狀態的 dict，沒有就回傳空 dict。

        這是選用的：板子讀不到這些資訊時維持預設實作即可。
        鍵名建議用 MowerStatus.msg 的欄位名（battery_voltage / battery_current /
        is_overheated ...），上層才好直接對應。
        """
        return {}
