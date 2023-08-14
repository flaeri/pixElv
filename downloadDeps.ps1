$ProgressPreference = 'SilentlyContinue'
Invoke-WebRequest -Uri "https://otterbro.com/pixElv-deps.7z" -OutFile "pixElv-deps.7z"
$ProgressPreference = 'Continue'
7z x .\pixElv-deps.7z -o"./deps/"