import sys
if sys.version_info.major < 3:
   print("Use python3 to run this file!")
   exit()

class Colors:
   BLACK = "\033[0;30m"
   RED = "\033[0;31m"
   GREEN = "\033[0;32m"
   BROWN = "\033[0;33m"
   BLUE = "\033[0;34m"
   PURPLE = "\033[0;35m"
   CYAN = "\033[0;36m"
   LIGHT_GRAY = "\033[0;37m"
   DARK_GRAY = "\033[1;30m"
   LIGHT_RED = "\033[1;31m"
   LIGHT_GREEN = "\033[1;32m"
   YELLOW = "\033[1;33m"
   LIGHT_BLUE = "\033[1;34m"
   LIGHT_PURPLE = "\033[1;35m"
   LIGHT_CYAN = "\033[1;36m"
   LIGHT_WHITE = "\033[1;37m"
   BOLD = "\033[1m"
   FAINT = "\033[2m"
   ITALIC = "\033[3m"
   UNDERLINE = "\033[4m"
   BLINK = "\033[5m"
   NEGATIVE = "\033[7m"
   CROSSED = "\033[9m"
   END = "\033[0m"


def runCommand(command:list, internal:bool = False, requireConfirm:bool = False)->bool:
   """Run Commands

   Args:
       command (list): Args passed for shell running
       internal (bool, optional): True indicate this is an internal command, if set to true, will directly run command without bothering user. Defaults to False.
         This confict with requireConfirm
       requireConfirm (bool, optional): Require user to input y/n for confirming. Defaults to False.

   Returns:
       bool: If requireConfirm is true, will return True if user decide to keep running script, else false 
   """
   if requireConfirm:
      user_said = False
      user_option = False
      while (not user_said):
         print(Colors.YELLOW + " Confirm running following command, answer y or n:")
         print(command)
         print("answer y or n: " + Colors.END)
         answer = input().lower()
         if answer == "y":
            user_said = True
            user_option = True
         elif answer == "n":
            user_said = True
            print(Colors.RED + "You have refused running the command\n", end="")
            
      if user_option:
         print(Colors.GREEN + "Running ", end="")
         print(command)
         print(Colors.END)
         subprocess.call(command)
      return user_option
   
   if not internal:
      print(Colors.GREEN + "Running ", end="")
      print(command)
      print(Colors.END)
   subprocess.call(command)
   return True


import os
import subprocess

if (os.getenv('CI')):
   # Exit now because in CI no need to install CAN_COM_HUB driver
   print(Colors.LIGHT_GREEN+"Skipping installing udev rules for CAN_COM_HUB because we are in CI "+Colors.END)
   exit()

# start install stuff here
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))

print(Colors.GREEN + "************************************************\nCopy udev rules for CAN_COM_HUB"+Colors.END)
print(Colors.YELLOW + "Installing udev rules for you, PLEASE type you password\nRefusing to run this step will make you unable to use your robot"+Colors.END)
if (not os.path.exists("/etc/udev/rules.d/CAN_COM_HUB.rules")):
   print("Start to copy CAN2COM_HUB.rules")
else:
   print(Colors.YELLOW+Colors.BLINK + "CAN2COM_HUB.rules existed, will overwrite file"+Colors.END)
   runCommand(["sudo", "rm", "/etc/udev/rules.d/CAN_COM_HUB.rules"],internal=True)

runCommand(["sudo", "cp", os.path.join(CURRENT_DIR, "CAN_COM_HUB.rules"), "/etc/udev/rules.d/CAN_COM_HUB.rules"],internal=True)
print(Colors.GREEN + "Finish copy"+Colors.END)
runCommand(["sudo", "systemctl", "stop", "nvgetty"],internal=True)
runCommand(["sudo", "service", "udev", "reload"],internal=True)
runCommand(["sudo", "service", "udev", "restart"],internal=True)
print(Colors.GREEN + "************************************************"+Colors.END)
