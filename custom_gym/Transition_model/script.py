import subprocess
# subprocess.call(('python', input('enter script name: ')))
path = "drone_3d_trajectory_following.py"
subprocess.call(('python3', path))


input('press ENTER to kill me')
