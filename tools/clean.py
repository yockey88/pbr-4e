import os
import sys
import shutil
import project_settings

args = project_settings.ProcessArguments(sys.argv)

if not os.path.exists("bin/Debug/{}".format(project_settings.EXE_NAME)) and not os.path.exists("bin/Release/{}".format(project_settings.EXE_NAME)):
    print("No project to clean")
    sys.exit(1)

print("Cleaning project")
print("\n")

if os.path.exists("bin/Debug/{}".format(project_settings.EXE_NAME)):
    shutil.rmtree("bin/Debug/{}".format(project_settings.EXE_NAME))
    shutil.rmtree("bin_obj/Debug/{}".format(project_settings.EXE_NAME))

if os.path.exists("bin/Release/{}".format(project_settings.EXE_NAME)):
    shutil.rmtree("bin/Release/{}".format(project_settings.EXE_NAME))
    shutil.rmtree("bin_obj/Release/{}".format(project_settings.EXE_NAME))

sys.exit(0)
