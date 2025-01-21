# -*- coding: utf-8 -*-

# Copyright Keysight Technologies 2017 - 2020

def main():
	import os, sys
	varDictionary={}
	exprDictionary={}
	ADS_HPEESOF_DIR=r"C:\Program Files\Keysight\ADS2024"
	libName=r"ant_lib"
	substName=r"Substrat_ant"
	workspacePath=r"C:\Users\tony\ant_wrk"
	libdefs=r"lib.defs"
	envEMPROHOME=r"C:\Program Files\Keysight\ADS2024\fem\2024.00/Win32_64/bin"
	verbose=False
	sys.path.append(os.path.join(envEMPROHOME))
	import empro.toolkit.via_designer as via_designer

	via_designer.mainGui.main(varDictionary, exprDictionary, ADS_HPEESOF_DIR, libName, substName, workspacePath=workspacePath, libdefs=libdefs, verbose=verbose)

if __name__=="__main__":
	main()
