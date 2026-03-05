@echo off
cd /d c:\Users\patje\Documents\esp\play_mp3_control
call C:\esp\v5.5.3\esp-idf\install.bat
call C:\esp\v5.5.3\esp-idf\export.bat
idf.py build
