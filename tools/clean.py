import os
import sys
import shutil
import project_settings

args = project_settings.ProcessArguments(sys.argv)
PROJECT = project_settings.GetArgValue(args, "p", project_settings.EXE_NAME)

if not os.path.exists("bin/Debug/{}".format(PROJECT)) and not os.path.exists("bin/Release/{}".format(PROJECT)):
    print("No project to clean")
    sys.exit(1)

print("Cleaning project")
print("\n")

if os.path.exists("bin/Debug/{}".format(PROJECT)):
    shutil.rmtree("bin/Debug/{}".format(PROJECT))
    shutil.rmtree("bin_obj/Debug/{}".format(PROJECT))

if os.path.exists("bin/Release/{}".format(PROJECT)):
    shutil.rmtree("bin/Release/{}".format(PROJECT))
    shutil.rmtree("bin_obj/Release/{}".format(PROJECT))

sys.exit(0)
