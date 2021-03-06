'''
MTRN4110 Group 13 'Lucky 13'
Run this python script to execute everything
'''

import subprocess as sp
import argparse, codecs, re

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

def launch_webots(x=0):
    print("Launching webots world...")
    if x == 0:
        sp.run(R"phase_d\world\webots_world.wbt", shell=True)
    elif x == 1:
        sp.run(R"phase_d\world\webots_world_features.wbt", shell=True)
    elif x == 2:
        sp.run(R"phase_d\world\webots_world_features_obstacles.wbt", shell=True)
    elif x == 3:
        doc1()
        sp.run(R"phase_d\world\webots_world_allfeatures.wbt", shell=True)
    elif x == 4:
        doc2()
        sp.run(R"phase_d\world\webots_world_manual.wbt", shell=True)

def doc1():
    '''
    Once the robot has explored the map and returned to start, 
    pause the simulation and insert some obstacles
    '''
    print(doc1.__doc__)

def doc2():
    '''
    Click inside the webots window and use the arrow keys to navigate
    '''
    print(doc2.__doc__)

def main():

    print("Checking installation of your environment...")
    environment_status = check_environment()
    if environment_status == False:
        install_status = install_environment()
        if install_status == False:
            print("Installation Failed")
            exit(-1)
    
    else:
        parser = argparse.ArgumentParser()
        parser.add_argument('integer', type=int, default=[0], nargs='*')
        args = parser.parse_args()

        sp.run(R"echo Running programs", shell=True)
        if args.integer[0] == 0:
            convert_map()
        launch_webots(args.integer[0])

if __name__ == "__main__":
    main()
