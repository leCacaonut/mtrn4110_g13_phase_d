'''
MTRN4110 Group 13 'Lucky 13'
Run this python script to execute everything
'''

import subprocess as sp
import codecs, re

def check_environment():
    env_list = sp.check_output(["conda", "info", "--envs"])
    env_list = env_list.replace(b'\\', b'/')
    env_list = codecs.escape_decode(env_list)[0].decode("utf-8")
    
    env_var = re.split(R"\n|\r|/| ", env_list)
    
    if "mtrn4110_g13" in env_var:
        return True
    else:
        return False

def install_environment():
    install_process = sp.Popen(["conda", "create", "-n", "mtrn4110_g13", "python=3.7"])
    install_process.wait()
    env_status = check_environment()
    if env_status == False:
        return False
    else:
        req_process = sp.Popen(R"install_env.bat", shell=True)
        req_process.wait()
        return True

def convert_map():
    sp.run(R"echo Printing map...", shell=True)
    sp.run(R"run_phase_c.bat", shell=True)
    sp.run(R"echo Map exported", shell=True)

def launch_webots():
    print("Launching webots world...")
    sp.run(R"phase_d\world\webots_world_features_obstacles.wbt", shell=True)

def main():

    print("Checking installation of your environment...")
    environment_status = check_environment()
    if environment_status == False:
        install_status = install_environment()
        if install_status == False:
            print("Installation Failed")
            exit(-1)
    
    else:
        sp.run(R"echo Running programs", shell=True)
        convert_map()
        launch_webots()

if __name__ == "__main__":
    main()
