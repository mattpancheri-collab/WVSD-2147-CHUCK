@echo off
set "JAVA_HOME=C:\Users\Public\wpilib\2026\jdk"
set "PATH=%JAVA_HOME%\bin;%PATH%"
call gradlew.bat build
