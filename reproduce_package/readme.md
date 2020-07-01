# What's the package contains

* **coe.jar** : maestroV1 co-simulation engine
* **TrackingSimulator.json** : configuration used the the co-simulation engine

* **Three FMUs**
	* **fmus/BicycleDynamic.fmu** : Model of the Robotti's dynamics
	* **fmus/BicycleTracking.fmu** : Model tracking the dynamics of the Robotti and matches those by calibrating internal parameters
	* **fmus/BicycleDriver.fmu** : Provides control commands to 


# Pre-requisites
* Windows 10
* Java
* Python 3.8.2 installed in system path ( note that issues may arise if virtual environments are used )

For questions/assistance please use chat:

https://gitter.im/INTO-CPS/PyFMU

# Install

``` bash
pip install pyfmu fmpy scipy matplotlib oomodellingpython pandas
```



# Running

1. Modify the URIs in the *.json file to point to an absolute path to the FMUs on you system

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


# References

``` bibtex
@inproceedings{Legaard2020,
  title = {Rapid {{Prototyping}} of {{Self}}-{{Adaptive}}-{{Systems}} Using {{Python Functional Mockup Units}}},
  author = {Legaard, Christian Møldrup and Gomes, Cláudio and Larsen, Peter Gorm and Foldager, Frederik F.},
  date = {2020},
  pages = {to appear},
  publisher = {{ACM New York, NY, USA}},
  location = {{Virtual event}},
  eventtitle = {2020 {{Summer Simulation Conference}}},
  keywords = {business process model and notation,distributed simulation,high level architecture,model driven architecture},
  series = {{{SummerSim}} ’20}
}
```