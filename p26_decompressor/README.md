# wp3_decompressor
ROS node for receiving and decompressing 3D sensor data on compressed by the [wp3_compressor](https://github.com/SFI-Mechatronics/wp3_compressor) node. The work is based on the Point Cloud Library's octree compressor.

**Keywords:** ROS, RGB-D, decompression

### License
The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Joacim Dybedal<br />
Affiliation: [SFI Offshore Mechatronics](https://sfi.mechatronics.no/), [University of Agder](https://www.uia.no/en)<br />
Maintainer: Joacim Dybedal, joacim.dybedald@uia.no**

The wp3_decompressor package has been tested under ROS Kinetic Kame and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

If you use this work in an academic context, please cite the following publication:

* J. Dybedal, A. Aalerud and G. Hovland, **Embedded Processing and Compression of 3D Sensor Data for Large Scale Industrial Environments**. Submitted. to Sensors, 2018. (Accepted, awaiting publication)

        @article{Dybedal2019,
            author = {Dybedal, Joacim and Aalerud, Atle and Hovland, Geir},
            journal = {submitted to Sensors},
            title = {{Embedded Processing and Compression of 3D Sensor Data for Large Scale Industrial Environments}},
            year = {2019}
        }

## Installation
### Building from Source
#### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Point Cloud Library](http://www.pointclouds.org)

#### Building
To build from source, clone the latest version from this repository into your catkin workspace and compile the package using
````bash
cd catkin_workspace/src
git clone https://github.com/SFI-Mechatronics/wp3_decompressor.git
cd ../
catkin_make
````

## Usage
The decompressor can be started by using the **.launch** file found in the **launch/** folder. 
This contains parameters such as sensor name and filter value. The file must be adapted based on the sensors used by the compressor.

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/SFI-Mechatronics/wp3_decompressor/issues).
