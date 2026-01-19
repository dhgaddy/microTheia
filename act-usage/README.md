# Instructions

- https://avlsi.csl.yale.edu/act/doku.php?id=summer2024:dockersetup
- be in this folder
- run 
```bash
docker build -t cadtools - < Dockerfile
docker run -d -p 7500:22 -v /path/to/directory:/share cadtools
ssh -p 7500 user@localhost
```
- password for the user account user to userpass, and the root password to rootpass