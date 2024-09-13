**Welcome to the risk-aware-unmanned-ship-for-roc project!**

Please do you a favor and use Andaconda / miniconda to ease your installation.

### Setup Seachart with Anaconda (See [Seachart](https://github.com/simbli/seacharts))

Install an edition of the [Anaconda](
https://www.anaconda.com/products/individual-d) package manager, and then create a new
_conda environment_
with [Python 3.11](https://www.python.org/downloads/) or higher using e.g. the
graphical user interface of [PyCharm Professional](
https://www.jetbrains.com/lp/pycharm-anaconda/) as detailed [here](
https://www.jetbrains.com/help/pycharm/conda-support-creating-conda-virtual-environment.html
).

The required data processing libraries for spatial calculations and
visualization may subsequently be installed simply by running the following
commands in the terminal of your chosen environment:

```
conda install -c conda-forge fiona cartopy matplotlib
conda install matplotlib-scalebar cerberys pyyaml
conda install seachart
```

### Install Requirements

```
conda install --file requirements.txt
```
