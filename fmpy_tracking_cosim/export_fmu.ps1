$output = "TrackingSimulator"

Remove-Item $output"\resources" -Recurse
Remove-Item $output"\binaries" -Recurse

& pyfmu export -p .\TrackingSimulatorProject\ -o TrackingSimulator

$targetZip = $output+".zip"

Compress-Archive -Path $output"\*" -DestinationPath $targetZip -Force

$fmu = $output + ".fmu"

Remove-Item $fmu
Rename-Item $targetZip $output".fmu" -Force
