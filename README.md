# Fault Tolerant Cache Simulator, based on gem5 (https://www.gem5.org/)

[Gem5 simulator](https://www.gem5.org/), enchanced with some of the well known cache fault tolerant schemes [1][2][3][4]. 

This version has been branched out from the following gem5 commit id:
```sh
commit 9d3b9e96c56386ee6539657c21cba95e118e576a
Author: Gabe Black <gabeblack@google.com>
Date:   Tue Oct 15 21:48:31 2019 -0700
```

**The supported schemes are the following:**
 -	*Clean cache*
 -	*Faulty cache – Block disabling scheme [4]*
 -	*Faulty cache – Subblock disabling scheme (2/4/8 subblocks) [1]*
 -	*Faulty cache – MAEP scheme [2]*
 -	*Faulty cache – FTA scheme [3]*

The selection of each cache configuration can be done by modifing the cache config file (= configs/common/Caches.py). Details about the configuration can be found in the following table. 

| Scheme | Modification |
| ------ | ------ |
| Clean Cache | [No modification] |
| Block Disabling [4] | faulty_cache = True |
| SubBlock Disabling [1] | faulty_cache = True <br> sub_blocks = 2/4/8 |
| MAEP scheme [2] | faulty_cache = True <br> sub_blocks = 8 <br> maep_scheme = True |
| FTA scheme [3] |  faulty_cache = True <br> sub_blocks = 4/8 <br> fta_predictor = True <br> sampling_predictor = 1/2/4/8/32/64 |

**Directories:**
- *run_scripts*: You can find two sample bash scripts, which can be used to configure the simulator appropriately for each specific case. Scripts simulated SysCall Emulation over [polybench](https://web.cse.ohio-state.edu/~pouchet.2/software/polybench/) benchmark suite.
- *src/mem/cache/tags/fault_maps*: A small number of fault maps and three fault probabilities provided under this directory. Use different fault maps and various probability to simulate the desired fault scenario. 
- *src/mem/cache/predictor/*. This directory encapsulates the spatial locality predictor described in [3]. Also GC-SFP directories provides various predictor configurations. :

The implementation of this work was part of my phd, under the supervision of *prof. Dimitris Nikolos* and *prof. Georgios Keramidas*. 

*Michail Mavropoulos*
- [mavropoulo@ceid.upatras.gr](mailto:mavropoulo@ceid.upatras.gr)
- [http://students.ceid.upatras.gr/~mavropoulo/](http://students.ceid.upatras.gr/~mavropoulo/)

**References:**

[1] J. Abella, J. Carretero, P. Chaparro, X. Vera, and A. González. Low Vccmin Fault-Tolerant Cache with Highly Predictable Performance. Proc. of Intl. Symposium on Microarchitecture, 2009.

[2] Y.G. Choi, S. Yoo, S. Lee, and J.H. Ahn. Matching Cache Access Behavior and Bit Error Pattern for High Performance Low Vcc L1 Cache. Proc. of Design Automation Conference, 2011.

[3] M. Mavropoulos, G. Keramidas, and D. Nikolos. Enabling Efficient Sub-block Disabled Caches Using Coarse Grain Spatial Predictions. IEEE Transactions on Computers, 2021. (Under review, not published work)

[4] G.S. Sohi. Cache Memory Organization to Enhance the Yield of High-Performance VLSI Processors. Trans. on Computers, 1989. 
