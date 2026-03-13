import json

with open("filelist.json") as filelist:
    files = json.load(filelist)["files"]
    for f in files:
        print(f,end=" ")
    print()
