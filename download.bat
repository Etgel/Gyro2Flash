@echo off
cd /d %~dp0
IF EXIST ".\build\Gyro2Flash.bin" (
    copy ".\build\Gyro2Flash.bin" "D:\"
    echo File copied successfully.
) ELSE (
    echo File not exist.
)

