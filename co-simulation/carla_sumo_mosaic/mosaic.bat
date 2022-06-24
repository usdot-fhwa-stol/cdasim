@ECHO OFF
SetLocal EnableDelayedExpansion

REM set maximum JVM memory
set javaMemorySizeXmx=2G

REM uncomment to activate remote debugging
REM set javaRemoteDebugging=-agentlib:jdwp=transport=dt_socket,server=y,suspend=y,address=10000

set libs=

rem core components
set Filter= .\lib\mosaic\*.jar
for %%f in (%Filter%) do (
if not "!libs!" == "" set libs=!libs!;
    set libs=!libs!%%f
)

rem third-party components
set Filter= .\lib\third-party\*.jar
for %%f in (%Filter%) do (
if not "!libs!" == "" set libs=!libs!;
    set libs=!libs!%%f
)

set libs=!libs!;

java -Xmx%javaMemorySizeXmx% %javaRemoteDebugging% -cp !libs! org.eclipse.mosaic.starter.MosaicStarter %*

EndLocal

exit /b %errorlevel%
