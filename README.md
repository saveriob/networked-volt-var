networked-volt-var
==================

A comparison of Volt/VAR feedback control laws: fully decentralized vs networked strategies.

This repository contains the Matlab and MatPower source code used in the paper

S. Bolognani, R. Carli, G. Cavraro, and S. Zampieri,  
"On the need for communication for voltage regulation of power distribtuion grids",  
IEEE Transactions on Control of Network Systems, 6(3):1111-1123, 2019. [doi: 10.1109/TCNS.2019.2921268](http://doi.org/10.1109/TCNS.2019.2921268)


**You are encouraged to used this code to test you voltage regulation strategy and to replicate the results of the paper**

- comparison.m generates the main plots that have been used in the paper.
- case_ieee123_2compensators is the Matpower case file corresponding to a modified IEEE 123-bus test feeder
- pvproduction.mat contains the time series of the PV power production
- demandprofile.mat contains the time series of the power demands by the loads
- localstatic.m is an auxiliary function that implements fully decentralized control strategies
- gparameters.m is an auxiliary function that computes the interagent gains for the networked control strategy

Useful links:

- The original IEEE 123-bus test feeder is available at http://ewh.ieee.org/soc/pes/dsacom/testfeeders/index.html
- MatPower is available at http://www.pserc.cornell.edu/matpower/ and can be obtained from https://github.com/MATPOWER/matpower
