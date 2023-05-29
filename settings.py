import os
import platform


class Project_path:
    OS = platform.system()
    HOME = os.path.expanduser('~')
    BACKEND_BASE = os.getenv('PROJECT_DIR')
    # print("vdsfv",BACKEND_BASE)
    UAV = os.path.join(BACKEND_BASE, 'UAV')
    GCS = os.path.join(BACKEND_BASE, 'GCS')
    WEB_BASE = os.path.join(GCS, 'ADTL-ClientApp')
    SITL = os.path.join(BACKEND_BASE, 'SITL')
    

if __name__ == '__main__':
    # print(os.getenv('PROJECT_DIR'))
    print(Project_path.OS, Project_path.HOME, Project_path.WEB_BASE, Project_path.BACKEND_BASE)