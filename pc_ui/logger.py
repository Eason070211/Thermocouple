#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FileLogger —— 文件日志模块 (与 GUI 解耦, 可独立使用)

日志写到 <程序目录>/logs/ 下, 文件名 log_YYYYMMDD_HHMMSS.log (启动时间戳),
每条日志自带完整时间戳 [YYYY-MM-DD HH:MM:SS.mmm]。

用法::

    logger = FileLogger()                    # 默认 logs/ 目录
    logger = FileLogger("D:/my_logs")        # 自定义目录
    logger.write("串口已连接", "info")        # level: info / warn / error
    logger.close()
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional

# 级别前缀: INFO / WARN / ERROR, 在日志文件里一眼区分
_LEVEL_PREFIX = {"info": "INFO", "warn": "WARN", "error": "ERROR"}

# 级别数值: 数字越大越严重, 用于与 level 参数过滤比较
_LEVEL_VALUE = {"info": 0, "warn": 1, "error": 2}


class FileLogger:
    """线程安全的简单文件日志: 自动建目录、时间戳文件名、每条带时间戳、行缓冲。

    设计取舍:
      * 不引入 logging 标准库 —— 需求只要求"能用", 简单可靠优先;
      * ``buffering=1`` 是行缓冲, 每写一行就落盘, 程序异常退出也不丢日志;
      * 所有公开方法用锁保护, 串口线程/记录线程/UI 线程都能安全调用;
      * 磁盘出错时静默吞掉 (返回 False), 不允许日志系统把主程序拖垮。
    """

    def __init__(self, directory: Optional[str] = None,
                 prefix: str = "log", level: str = "info"):
        """
        :param directory: 日志目录; None = 程序目录下 ``logs/``
        :param prefix:    文件名前缀, 最终 ``<prefix>_YYYYMMDD_HHMMSS.log``
        :param level:     最低级别: "info" / "warn" / "error"
        """
        self.level = level
        self._lock = threading.Lock()
        self.path: Optional[str] = None
        self._fh = None

        if directory is None:
            # 脚本目录: pc_ui/logger.py -> pc_ui/
            directory = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), "logs")
        try:
            os.makedirs(directory, exist_ok=True)
            self.path = os.path.join(directory, "%s_%s.log"
                                     % (prefix, time.strftime("%Y%m%d_%H%M%S")))
            self._fh = open(self.path, "w", encoding="utf-8", buffering=1)
            self.write("日志文件: %s" % self.path, "info")
        except OSError:
            # 磁盘满 / 权限不足 -> 日志不可用, 但主程序照常跑
            self.path = None
            self._fh = None

    # ------------------------------------------------------------------
    def write(self, message: str, level: str = "info") -> bool:
        """写一条日志 (自动加完整时间戳 + 级别前缀)。返回是否成功。"""
        if level not in _LEVEL_PREFIX:
            level = "info"
        if _LEVEL_VALUE[level] < _LEVEL_VALUE[self.level]:
            return False
        if self._fh is None:
            return False
        line = "[%s.%03d] [%s] %s" % (
            time.strftime("%Y-%m-%d %H:%M:%S"),
            int((time.time() % 1) * 1000),
            _LEVEL_PREFIX[level],
            message)
        try:
            with self._lock:
                self._fh.write(line + "\n")
            return True
        except (OSError, ValueError):
            return False

    def close(self) -> None:
        """关闭日志文件 (安全, 可重复调用)。"""
        with self._lock:
            if self._fh is not None:
                try:
                    self._fh.flush()
                    self._fh.close()
                except (OSError, ValueError):
                    pass
                self._fh = None


if __name__ == "__main__":
    # 独立自检: python -m pc_ui.logger  (用工作区临时目录, 兼容受限沙盒环境)
    import shutil
    tmp = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_logger_selftest")
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp, exist_ok=True)
    try:
        logger = FileLogger(tmp)
        ok1 = logger.write("测试信息", "info")
        ok2 = logger.write("测试警告", "warn")
        ok3 = logger.write("测试错误", "error")
        path = logger.path
        logger.close()
        lines = open(path, encoding="utf-8").read().splitlines()
        print("path=%s" % path)
        print("\n".join(lines))
        assert ok1, ok1
        assert ok2, ok2
        assert ok3, ok3
        assert len(lines) == 4, lines          # 1 行文件路径 + 3 条日志
        for ln in lines[1:]:
            assert ln.startswith("[20") and "] [" in ln, ln
        print("logger 自检: 全部通过")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
