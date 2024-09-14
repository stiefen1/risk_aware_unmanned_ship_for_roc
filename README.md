**Welcome to the risk-aware-unmanned-ship-for-roc project!**

Make sure to clone this repository in recursive mode:

```
git clone --recurse-submodules https://github.com/stiefen1/risk_aware_unmanned_ship_for_roc.git
```

Also, do you a favor and use conda to ease your installation.

### Setup Seachart with Anaconda (See [Seachart](https://github.com/simbli/seacharts))

Install an edition of the [Anaconda](
https://www.anaconda.com/products/individual-d) package manager, activate your base environment (```conda activate```) and navigate to this folder. To create and setup your environment, just run the following command and go grab a cup of coffee.

```
conda env create -f environment.yml
```
Once done, verify that the environment  ```risk-aware-unmanned-ship-for-roc-env``` appears in the list when typing:

```
conda env list
```
And then activate it:
```
conda activate risk-aware-unmanned-ship-for-roc-env
```

To check that everything was correctly installed you can run two python files (one for each of the two main dependencies [SeaCharts](https://github.com/simbli/seacharts/tree/main) and [ship_in_transit_simulator](https://github.com/BorgeRokseth/ship_in_transit_simulator/tree/master)). Make sure that you have already bathymetry data in the `/data` folder, following instructions from [SeaCharts](https://github.com/simbli/seacharts/tree/main) package.

```
python -m tests.seacharts.display
python -m tests.ship_in_transit_simulator.ship_route_following
```


