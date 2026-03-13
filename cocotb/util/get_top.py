import json

with open("filelist.json") as filelist:
    print(json.load(filelist)["top"])
