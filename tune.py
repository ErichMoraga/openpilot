from selfdrive.kegman_conf import kegman_conf
import subprocess
import os
from common.params import Params

BASEDIR = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../"))

letters = { "a":[ "###", "# #", "###", "# #", "# #"], "b":[ "###", "# #", "###", "# #", "###"], "c":[ "###", "#", "#", "#", "###"], "d":[ "##", "# #", "# #", "# #", "##"], "e":[ "###", "#", "###", "#", "###"], "f":[ "###", "#", "###", "#", "#"], "g":[ "###", "# #", "###", "  #", "###"], "h":[ "# #", "# #", "###", "# #", "# #"], "i":[ "###", " #", " #", " #", "###"], "j":[ "###", " #", " #", " #", "##"], "k":[ "# #", "##", "#", "##", "# #"], "l":[ "#", "#", "#", "#", "###"], "m":[ "# #", "###", "###", "# #", "# #"], "n":[ "###", "# #", "# #", "# #", "# #"], "o":[ "###", "# #", "# #", "# #", "###"], "p":[ "###", "# #", "###", "#", "#"], "q":[ "###", "# #", "###", "  #", "  #"], "r":[ "###", "# #", "##", "# #", "# #"], "s":[ "###", "#", "###", "  #", "###"], "t":[ "###", " #", " #", " #", " #"], "u":[ "# #", "# #", "# #", "# #", "###"], "v":[ "# #", "# #", "# #", "# #", " #"], "w":[ "# #", "# #", "# #", "###", "###"], "x":[ "# #", " #", " #", " #", "# #"], "y":[ "# #", "# #", "###", "  #", "###"], "z":[ "###", "  #", " #", "#", "###"], " ":[ " "], "1":[ " #", "##", " #", " #", "###"], "2":[ "###", "  #", "###", "#", "###"], "3":[ "###", "  #", "###", "  #", "###"], "4":[ "#", "#", "# #", "###", "  #"], "5":[ "###", "#", "###", "  #", "###"], "6":[ "###", "#", "###", "# #", "###"], "7":[ "###", "  # ", " #", " #", "#"], "8":[ "###", "# #", "###", "# #", "###"], "9":[ "###", "# #", "###", "  #", "###"], "0":[ "###", "# #", "# #", "# #", "###"], "!":[ " # ", " # ", " # ", "   ", " # "], "?":[ "###", "  #", " ##", "   ", " # "], ".":[ "   ", "   ", "   ", "   ", " # "], "]":[ "   ", "   ", "   ", "  #", " # "], "/":[ "  #", "  #", " # ", "# ", "# "], ":":[ "   ", " # ", "   ", " # ", "   "], "@":[ "###", "# #", "## ", "#  ", "###"], "'":[ " # ", " # ", "   ", "   ", "   "], "#":[ " # ", "###", " # ", "###", " # "], "-":[ "  ", "  ","###","   ","   "] }
# letters stolen from here: http://www.stuffaboutcode.com/2013/08/raspberry-pi-minecraft-twitter.html

def print_letters(text):
    bigletters = []
    for i in text:
        bigletters.append(letters.get(i.lower(),letters[' ']))
    output = ['']*5
    for i in range(5):
        for j in bigletters:
            temp = ' '
            try:
                temp = j[i]
            except:
                pass
            temp += ' '*(5-len(temp))
            temp = temp.replace(' ',' ')
            temp = temp.replace('#','\xE2\x96\x88')
            output[i] += temp
    return '\n'.join(output)

import sys, termios, tty, os, time

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

button_delay = 0.2

kegman = kegman_conf()
#kegman.conf['tuneGernby'] = "1"
#kegman.write_config(kegman.conf)
param = ["innerLoopGain", "outerLoopGain", "timeConstant", "actuatorEffectiveness"]

try:
  devnull = open(os.devnull, 'w')
  text_file = open("/data/username", "r")
  if text_file.mode == "r":
    user_name = text_file.read()
    if (user_name == ""):
      sys.exit()
    '''cmd = '/usr/local/bin/python /data/openpilot/dashboard.py'
    process = subprocess.Popen(cmd, shell=True,
                               stdout=devnull,
                               stderr=None,
                               close_fds=True)'''
  text_file.close()
except:
  try:
    user_name = raw_input('Username: ').strip() if sys.version_info.major == 2 else input('Username: ').strip()
    text_file = open("/data/username", "w")
    text_file.write(user_name)
    text_file.close()
    sys.exit()
  except:
    params = Params()
    user_name = params.get("DongleId")

cmd = '/usr/local/bin/python /data/openpilot/dashboard.py'
process = subprocess.Popen(cmd, shell=True,
                           stdout=subprocess.PIPE,
                           stderr=None,
                           close_fds=True)

j = 0

while True:
  print ""
  print print_letters(param[j][0:9])
  print ""
  print print_letters(kegman.conf[param[j]])
  print ""
  print ""
  print ("Press 1, 3, 5, 7 to incr 0.1, 0.05, 0.01, 0.001")
  print ("press a, d, g, j to decr 0.1, 0.05, 0.01, 0.001")
  print ("press 0 / L to make the value 0 / 1")
  print ("press SPACE / m for next /prev parameter")
  print ("press z to quit")

  char  = getch()
  write_json = False
  if (char == "7"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) + 0.001)
    write_json = True

  if (char == "5"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) + 0.01)
    write_json = True

  elif (char == "3"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) + 0.05)
    write_json = True

  elif (char == "1"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) + 0.1)
    write_json = True

  elif (char == "j"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) - 0.001)
    write_json = True

  elif (char == "g"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) - 0.01)
    write_json = True

  elif (char == "d"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) - 0.05)
    write_json = True

  elif (char == "a"):
    kegman.conf[param[j]] = str(float(kegman.conf[param[j]]) - 0.1)
    write_json = True

  elif (char == "0"):
    kegman.conf[param[j]] = "0"
    write_json = True

  elif (char == "l"):
    kegman.conf[param[j]] = "1"
    write_json = True

  elif (char == " "):
    if j < len(param) - 1:
      j = j + 1
    else:
      j = 0

  elif (char == "m"):
    if j > 0:
      j = j - 1
    else:
      j = len(param) - 1

  elif (char == "z"):
    process.kill()
    break

  if float(kegman.conf['timeConstant']) < 0 and float(kegman.conf['timeConstant']) != -1:
    kegman.conf['timeConstant'] = "0"

  if float(kegman.conf['timeConstant']) > 3:
    kegman.conf['timeConstant'] = "3"

  if float(kegman.conf['actuatorEffectiveness']) < 0 and float(kegman.conf['actuatorEffectiveness']) != -1:
    kegman.conf['actuatorEffectiveness'] = "0"

  if float(kegman.conf['actuatorEffectiveness']) > 3:
    kegman.conf['actuatorEffectiveness'] = "3"

  if float(kegman.conf['outerLoopGain']) < 0 and float(kegman.conf['outerLoopGain']) != -1:
    kegman.conf['outerLoopGain'] = "0"

  if float(kegman.conf['outerLoopGain']) > 10:
    kegman.conf['outerLoopGain'] = "10"

  if float(kegman.conf['innerLoopGain']) < 0 and float(kegman.conf['innerLoopGain']) != -1:
    kegman.conf['innerLoopGain'] = "0"

  if float(kegman.conf['innerLoopGain']) > 10:
    kegman.conf['innerLoopGain'] = "10"

  if write_json:
    kegman.write_config(kegman.conf)

  time.sleep(button_delay)
else:
  process.kill()
