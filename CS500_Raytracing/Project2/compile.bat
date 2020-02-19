rem *******************************************************************************
rem *** Make sure that you are using the Visual Studio Developer Command Prompt ***
rem *******************************************************************************
cl /EHsc /I. /DVERBOSE RayCasterDriver.cpp RayCaster.cpp DPmesh.cpp opengl32.lib lib/*.lib /link /subsystem:console

