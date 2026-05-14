#!/usr/bin/env python3
"""Auto-generate version header. Format: YYYY.MM.DD.TOTAL.DAILY"""

import json
import os
from datetime import date

STATE_FILE = os.path.join(os.path.dirname(__file__), "version_state.json")
VERSION_H  = os.path.join(os.path.dirname(__file__), "main", "include", "version.h")

today = date.today().isoformat()

if os.path.exists(STATE_FILE):
    with open(STATE_FILE) as f:
        state = json.load(f)
else:
    state = {"total": 0, "date": "", "daily": 0}

if state.get("date") == today:
    state["daily"] += 1
else:
    state["date"] = today
    state["daily"] = 1
state["total"] += 1

with open(STATE_FILE, "w") as f:
    json.dump(state, f, indent=2)

parts = today.split("-")
version_str = f"{parts[0]}.{parts[1]}.{parts[2]}.{state['total']}.{state['daily']}"

os.makedirs(os.path.dirname(VERSION_H), exist_ok=True)
with open(VERSION_H, "w") as f:
    f.write(f'#pragma once\n')
    f.write(f'#define AATGO_FW_VERSION_STR "{version_str}"\n')

print(f"Version: {version_str}")
