$env:RUST_BACKTRACE="full"

Push-Location C:\Users\clega\Desktop\pyfmu

pyfmu export --project .\examples\projects\BicycleDriver --output .\examples\exported\BicycleDriver
pyfmu export --project .\examples\projects\BicycleDynamic --output .\examples\exported\BicycleDynamic
pyfmu export --project .\examples\projects\BicycleTracking --output .\examples\exported\BicycleTracking

Pop-Location

Remove-Item -Force .\coe.log
Remove-Item -Force .\output.csv

$Config = ".\tracking_scenario_pyfmu_christian.json"
# $Config = ".\bycicle_driver_scenario.json"
# $Config = ".\tracking_scenario_open_modelica.json"
# $Config = ".\bycicle_driver_scenario_openmodelica.json"

java -jar "C:\Users\clega\Desktop\pyfmu\src\pyfmu\resources\validation\coe.jar" -v --configuration "$Config" --oneshot --starttime 0.0 --endtime 25.0 -v

python .\plot.py
