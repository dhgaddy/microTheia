# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 Group G Contributors
import json

with open("filelist.json") as filelist:
    files = json.load(filelist)["files"]
    for f in files:
        print(f,end=" ")
    print()
