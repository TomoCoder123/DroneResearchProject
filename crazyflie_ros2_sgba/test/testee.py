from tester import poop, prevPP
from dataclasses import dataclass

@dataclass
class ControlCommands_t:
  roll: float = 0
  pitch: float = 0
  yaw: float = 0
  altitude: float = 0

poop.stat += 2


pooper = poop()
pooper.a()
pooper.a()
print(pooper.tryer())
cmd = ControlCommands_t()
print(cmd)
print(prevPP())
print(prevPP())
print(prevPP())