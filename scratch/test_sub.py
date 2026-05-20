import subprocess
import os

home = os.path.expanduser('~')
p = subprocess.Popen(
    'make px4_sitl gz_x500', 
    shell=True, 
    cwd=os.path.join(home, 'PX4-Autopilot'), 
    stdout=subprocess.PIPE, 
    stderr=subprocess.STDOUT
)
try:
    out, _ = p.communicate(timeout=10)
    print(out.decode('utf-8'))
except subprocess.TimeoutExpired:
    p.kill()
    print("Process running fine (timed out).")
