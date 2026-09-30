#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""支持 `python -m pc_ui` 直接启动界面。"""

import sys

try:
    from .main import main
except ImportError:                   # pragma: no cover
    from main import main

if __name__ == "__main__":
    sys.exit(main())
