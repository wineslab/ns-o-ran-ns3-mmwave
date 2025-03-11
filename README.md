# ns3-mmwave fork for ns-O-RAN

================================

This repository is a fork of the [ns3-mmwave project](https://github.com/nyuwireless-unipd/ns3-mmwave) with updates to make it work with the [ns3-o-ran-e2](https://github.com/o-ran-sc/sim-ns3-o-ran-e2) ns-3 module.

This module enables the support for running multiple terminations of an O-RAN-compliant E2 interface inside the simulation process. It has been developed as part of a collaborative effort between the [Institute for the Wireless Internet of Things (WIoT)](https://wiot.northeastern.edu) at Northeastern University, Sapienza University of Rome, Mavenir, and the University of Padova.

## How to use

Please refer to this [quick start guide](https://openrangym.com/tutorials/ns-o-ran) that presents a tutorial to bridge ns-O-RAN and Colosseum RIC (i.e., OSC RIC bronze reduced) ns-O-RAN.

Additional material:

- Framework presentation https://openrangym.com/ran-frameworks/ns-o-ran 
- Tutorial OSC RIC version E ns-O-RAN connection  https://www.nsnam.org/tutorials/consortium23/oran-tutorial-slides-wns3-2023.pdf 
- Recording of the tutorial OSC RIC version E done at the WNS3 2023 https://vimeo.com/867704832 
- xApp repositories working with ns-O-RAN:
  - https://github.com/wineslab/ns-o-ran-scp-ric-app-kpimon 
  - https://github.com/wineslab/ns-o-ran-xapp-rc 
- Gymnasium Environment wrapper for ns-O-RAN https://github.com/wineslab/ns-o-ran-gym-environment

## How to contribute

See the [contributing](./CONTRIBUTING.md) page.

## References

More information can be found in the technical paper:

> A. Lacava, M. Bordin, M. Polese, R. Sivaraj, T. Zugno, F. Cuomo, and T. Melodia. "ns-O-RAN: Simulating O-RAN 5G Systems in ns-3", Proceedings of the 2023 Workshop on ns-3 (2023), [DOI:10.1145/3592149.3592161](https://dl.acm.org/doi/abs/10.1145/3592149.3592161) 

If you use the scenario-one.cc or the traffic steering implementation please cite:

>A. Lacava, M. Polese, R. Sivaraj, R. Soundrarajan, B. Bhati, T. Singh, T. Zugno, F. Cuomo, and T. Melodia. "Programmable and Customized Intelligence for Traffic Steering in 5G Networks Using Open RAN Architectures", IEEE Transactions on Mobile Computing (2024), [DOI:10.1109/TMC.2023.3266642](https://doi.org/10.1109/TMC.2023.3266642) [pdf](https://ieeexplore.ieee.org/document/10102369) [bibtex](https://ece.northeastern.edu/wineslab/wines_bibtex/andrea/LacavaAMC22.txt)


# mmWave ns-3 module #

This is an [ns-3](https://www.nsnam.org "ns-3 Website") module for the simulation
of 5G cellular networks operating at mmWaves. A description of this module can be found in [this paper](https://ieeexplore.ieee.org/document/8344116/ "mmwave paper").

Main features:

* Support of a wide range of channel models, including the model based on 3GPP TR 38.901 for frequencies between 0.5 and 100 GHz. Ray tracing and measured traces can also be used.

* Custom PHY and MAC classes supporting the 3GPP NR frame structure and numerologies.

* Custom schedulers for supporting dynamic TDD formats

* Carrier Aggregation at the MAC layer

* Enhancements to the RLC layer with re-segmentation of packets for retransmissions

* Dual Connectivity with LTE base stations, with fast secondary cell handover and channel tracking

* Simulation of core network elements (with also the MME as a real node)

## Installation
This repository contains a complete ns-3 installation with the addition of the mmwave module. 

Use these commands to download and build `ns3-mmwave`:
```
git clone https://github.com/nyuwireless-unipd/ns3-mmwave.git
cd ns3-mmwave
./ns3 configure --disable-python --enable-examples && ./ns3 build
```

## Usage example
You can use the following command to run the `mmwave-simple-epc` example. 
```
./ns3 --run mmwave-simple-epc
```
Other examples are included in `src/mmwave/examples/`

## Documentation
The documentation of this module is available at [this link](./src/mmwave/doc/mmwave-doc.md).

## Related modules
- MilliCar is an ns-3 module for the simulation of mmWave NR V2X networks. Check [this repo](https://github.com/signetlabdei/millicar) for further details.
- A seperate module is being developed for [mmWave UE Energy Consumption](https://github.com/arghasen10/mmwave-energy "mmwave-energy"). You can use this module for analyzing 
Energy Consumption behaviour of mmwave UE. Check this repository for further details.
- `ns3-mmwave-iab` is an extended version of `ns3-mmWave` adding wireless relaying capabilities to an ns-3 NetDevice, and the possibility of simulating in-band relaying at mmWave frequencies. Check [this repo](https://github.com/signetlabdei/ns3-mmwave-iab) for further details.

## References 
The following papers describe in detail the features implemented in the mmWave
module:
- [End-to-End Simulation of 5G mmWave Networks](https://ieeexplore.ieee.org/document/8344116/ "comst paper") is a comprehensive tutorial with a detailed description of the whole module. We advise the researchers interested in this module to start reading from this paper;
- [Integration of Carrier Aggregation and Dual Connectivity for the ns-3 mmWave Module](https://arxiv.org/abs/1802.06706 "wns3 2018") describes the Carrier Aggregation implementation;
- [Implementation of A Spatial Channel Model for ns-3](https://arxiv.org/abs/2002.09341 "wns3 2020") describes the integration of the spatial channel model based on the 3GPP specifications TR 38.901 V15.0.0;
- [Performance Comparison of Dual Connectivity and Hard Handover for LTE-5G Tight Integration](https://arxiv.org/abs/1607.05425 "simutools paper") describes the Dual Connectivity feature.

These other papers describe features that were implemented in older releases: 
- [ns-3 Implementation of the 3GPP MIMO Channel Model for Frequency Spectrum above 6 GHz](https://dl.acm.org/citation.cfm?id=3067678 "wns3 2017") describes the implementation of the 3GPP channel model based on TR 38.900;
- [Multi-Sector and Multi-Panel Performance in 5G mmWave Cellular Networks](https://arxiv.org/abs/1808.04905 "globecom2018") describes the multi-sector addition to the 3GPP channel model;

If you use this module in your research, please cite:

M. Mezzavilla, M. Zhang, M. Polese, R. Ford, S. Dutta, S. Rangan, M. Zorzi, _"End-to-End Simulation of 5G mmWave Networks,"_ in IEEE Communications Surveys & Tutorials, vol. 20, no. 3, pp. 2237-2263, thirdquarter 2018. [bibtex available here](https://ieeexplore.ieee.org/document/8344116/)

## Future work
We are actively developing new features for the mmWave module, including:
- 3GPP NR beam tracking
- 3GPP NR Integrated Access and Backhaul feature (see [this repo](https://github.com/signetlabdei/ns3-mmwave-iab) for more details)

## About
This module is being developed by [NYU Wireless](http://wireless.engineering.nyu.edu/) and the [University of Padova](http://mmwave.dei.unipd.it/).
This  work  was  supported  in  part by  the  U.S.  Department  of  Commerce  National  Institute  of  Standards  and Technology through the Project “An End-to-End Research Platform for Public Safety  Communications  above  6  GHz”  under  Award  70NANB17H16.



<!-- The new-handover branch offers integration between LTE and mmWave and dual connectivity features.
 -->

## Authors ##

The ns-3 mmWave module is the result of the development effort carried out by different people. The main contributors are: 
- Tommaso Zugno, University of Padova
- Michele Polese, University of Padova
- Matteo Pagin, University of Padova
- Mattia Lecci, University of Padova
- Matteo Drago, University of Padova
- Mattia Rebato, University of Padova
- Menglei Zhang, NYU Wireless
- Marco Giordani, University of Padova
- Marco Mezzavilla, NYU Wireless
- Sourjya Dutta, NYU Wireless
- Russell Ford, NYU Wireless
- Gabriel Arrobo, Intel

## License ##

This software is licensed under the terms of the GNU GPLv2, as like as ns-3. See the LICENSE file for more details.
