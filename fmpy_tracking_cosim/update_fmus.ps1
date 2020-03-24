Function Create-FMU{
    param(
        $directory
    )
    $targetZip = $directory+".zip"
    Compress-Archive -Path $directory"\*" -DestinationPath $targetZip -Force
    $fmu = $directory + ".fmu"
    Remove-Item $fmu
    Rename-Item $targetZip $fmu -Force
}

& pyfmu export -p .\TrackingSimulatorProject\ -o TrackingSimulator
Create-FMU TrackingSimulator
Remove-Item TrackingSimulator -Recurse

Create-FMU robotti_global

Create-FMU Steering_input
