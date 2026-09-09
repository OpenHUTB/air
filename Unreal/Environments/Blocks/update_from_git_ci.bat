@echo off
REM //---------- set up variable ----------
setlocal
set ROOT_DIR=%~dp0

set AirSimPath=%1

REM default path works for Blocks environment
if "%AirSimPath%"=="" set "AirSimPath=..\..\.."

IF NOT EXIST "%AirSimPath%" (
	echo "AirSimPath %AirSimPath% was not found"
	goto :failed
)

echo Using AirSimPath = %AirSimPath%

robocopy /MIR "%AirSimPath%\Unreal\Plugins\AirSim" Plugins\AirSim /XD temp *. /njh /njs /ndl /np
robocopy /MIR "%AirSimPath%\AirLib" Plugins\AirSim\Source\AirLib /XD temp *. /njh /njs /ndl /np

if not exist Plugins\Carla (
    if exist Plugins\AirSim\Source\SimWorldGameMode.h ren Plugins\AirSim\Source\SimWorldGameMode.h SimWorldGameMode.h.disabled
    if exist Plugins\AirSim\Source\SimWorldGameMode.cpp ren Plugins\AirSim\Source\SimWorldGameMode.cpp SimWorldGameMode.cpp.disabled
)

robocopy  /njh /njs /ndl /np "%AirSimPath%\Unreal\Environments\Blocks" "." *.bat 
robocopy  /njh /njs /ndl /np "%AirSimPath%\Unreal\Environments\Blocks" "." *.sh  
rem robocopy /njh /njs /ndl /np "%AirSimPath%" "." *.gitignore

REM cmd /c clean.bat
REM cmd /c GenerateProjectFiles.bat

goto :done

:failed
echo Error occured while updating.
exit /b 1

:done
REM if "%1"=="" pause