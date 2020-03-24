$output = "TrackingSimulator"

Remove-Item $output"\resources" -Recurse
Remove-Item $output"\binaries" -Recurse

& pyfmu export -p .\TrackingSimulatorProject\ -o $output

$targetZip = $output+".zip"

Compress-Archive -Path $output"\*" -DestinationPath $targetZip -Force

$fmu = $output + ".fmu"

Remove-Item $fmu
Rename-Item $targetZip $output".fmu" -Force

Remove-Item $output -Recurse