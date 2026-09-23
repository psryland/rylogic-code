#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
# Use:
#  RunUnitTests.py $(TargetPath) dependency1 dependency2 ...
import sys
import Rylogic as Tools

try:
	#sys.argv=["", "projects/rylogic/Rylogic.Core/bin/Debug/net10.0-windows/Rylogic.Core.dll",]
	
	#print(str(sys.argv))
	Tools.UnitTest(sys.argv[1], sys.argv[2:])

except Exception as ex:
	Tools.OnException(ex)