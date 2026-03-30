# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2024-2025 Group G Contributors
import json

with open("filelist.json") as filelist:
    print(json.load(filelist)["top"])
