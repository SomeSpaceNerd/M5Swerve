@echo off

"%~dp0.venv\Scripts\python.exe" "%~dp0bellman-framework\scripts\deploy_linux.py" "%~dp0deploy_linux.json"

exit /b %ERRORLEVEL%