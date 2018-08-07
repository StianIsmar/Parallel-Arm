'########################################################################

'##### @file: pkm_arm

'##### @version: 1.0.0

'##### @licence: MIT

'##### @author: Stian Ismar

'########################################################################
init:
SetCommand(_C,0)
main:

cmd = GetValue(_VAR, 9)
armSafety = GetValue(_DI,5)

if armSafety = 0 then
	print("The arm cannot move more CCW")
else
	setCommand(_P, cmd)
end if
goto main

'########################################################################
