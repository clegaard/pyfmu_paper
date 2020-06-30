# Pre-requisites
* Windows 10
* Java
* Python 3.8.2 installed in system path

# Install

``` bash
pip install pyfmu scipy matplotlib oomodellingpython pandas
```



# Running

1. Modify the URIs to point to an absolute path to the FMUs on you system

``` json
	"fmus": {
		"{src}": "file:///C:/Users/clega/Desktop/pyfmu/tracking_simulator_results/fmus/BicycleDriver.fmu",
		"{bd}": "file:///C:/Users/clega/Desktop/pyfmu/tracking_simulator_results/fmus/BicycleDynamic.fmu",
		"{track}": "file:///C:/Users/clega/Desktop/pyfmu/tracking_simulator_results/fmus/BicycleTracking.fmu"
	},
```

2. Run the simulate script. Note that the recalibration may take some time. This happens at around t=18.00 sec

``` bash
python simulate.py
```