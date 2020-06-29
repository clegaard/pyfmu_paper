$env:RUST_BACKTRACE="full"

Push-Location D:\srcctrl\github\pyfmu

pyfmu export --project .\examples\projects\BicycleDriver --output .\examples\exported\BicycleDriver
pyfmu export --project .\examples\projects\BicycleDynamic --output .\examples\exported\BicycleDynamic
pyfmu export --project .\examples\projects\BicycleTracking --output .\examples\exported\BicycleTracking

Pop-Location

Remove-Item -Force coe.log

$Config = ".\tracking_scenario.json"
# $Config = ".\bycicle_driver_scenario.json"

java -jar "D:\srcctrl\github\maestro\orchestration\coe\target\coe-1.0.11-SNAPSHOT-jar-with-dependencies.jar" -v --configuration "$Config" --oneshot --starttime 0.0 --endtime 25.0

python .\plot.py
