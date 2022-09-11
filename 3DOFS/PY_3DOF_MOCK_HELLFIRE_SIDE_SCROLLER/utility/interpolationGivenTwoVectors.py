import numpy as np

def linearInterpolation(x, xx, yy):
          lowIndex = None
          highIndex = None
          index = -1
          for number in xx:
                    index += 1
                    if number > x:
                              if highIndex == None:
                                        highIndex = index
                              elif number < xx[highIndex]:
                                        highIndex = index
                    if number < x:
                              if lowIndex == None:
                                        lowIndex = index
                              elif number > xx[lowIndex]:
                                        lowIndex = index
          if lowIndex == None:
                    lowIndex = 0
          if highIndex == None:
                    highIndex = -1
          # print(x, xx[lowIndex], xx[highIndex], yy[lowIndex], yy[highIndex])
          y = yy[lowIndex] + ((x - xx[lowIndex]) * ((yy[highIndex] - yy[lowIndex]) / (xx[highIndex] - xx[lowIndex])))
          return y