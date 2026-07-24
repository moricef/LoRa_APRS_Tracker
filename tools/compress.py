# Copyright (C) 2025 Ricardo Guzman - CA2RXU
# 
# This file is part of LoRa APRS Tracker.
# 
# LoRa APRS Tracker is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or 
# (at your option) any later version.
# 
# LoRa APRS Tracker is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with LoRa APRS Tracker. If not, see <https://www.gnu.org/licenses/>.

import gzip
import os
import datetime
import re
Import("env")

files = [
  'data_embed/index.html',
  'data_embed/script.js',
  'data_embed/style.css',
  'data_embed/bootstrap.js',
  'data_embed/bootstrap.css',
  'data_embed/favicon.png',
  'data_embed/github-sponsors.png',
  'data_embed/paypalme.png',
]

# Historical upstream CA2RXU firmware date restored from this fork's commits.
# Keep fixed: do not update it when bumping this fork/LVGL UI versions.
ca2rxu_firmware_version_date = "2026-01-12"
ui_version = "unknown"

with open('include/ui_common.h', encoding='utf-8') as header_file:
  for line in header_file:
    match = re.match(r'\s*#define\s+UI_VERSION\s+"([^"]+)"', line)
    if match:
      ui_version = match.group(1)
      break
       
for src in files:
  out = src + ".gz"
  
  
  with open(src, 'rb') as f:
    content = f.read()
    
  if src == 'data_embed/index.html':
    board_name = env.get("PIOENV", env["BOARD"])
    current_date = datetime.datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S') + " UTC"
    build_info = (
      f'Board / Environment = {board_name}<br>'
      f'Firmware (CA2RXU) version date: {ca2rxu_firmware_version_date}<br>'
      f'LVGL UI<br>'
      f'Version: {ui_version}<br>'
      f'Build date: {current_date}'
    ).encode()
    
    content = content.replace(b'%BUILD_INFO%', build_info)
      
  with open(out, 'wb') as f:
    f.write(gzip.compress(content, compresslevel=9))
