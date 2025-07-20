Usage
=====

Installation
------------

This library has been thoroughly tested with python 3.9 on a Raspberry Pi Zero 2W, running Raspberry Pi OS Bullseye (32-Bit). It should also be compatible with other Raspberry Pi models and operating systems with Python 3.9 or later, but no gurantees are made for those configurations.


It is recommended to use a virtual environment for your project to avoid conflicts with other packages. You can create a virtual environment using the following command:

.. code-block:: console

   (.venv) $ python3 -m venv .venv

Then, activate the virtual environment:

.. code-block:: console

   (.venv) $ source .venv/bin/activate

You can then install the library on your Raspberry Pi using pip:

.. code-block:: console

   (.venv) $ git clone https://github.com/Ballistyxx/blimp-utils.git
   cd blimp-utils
   sudo pip3 install .

Or, use the included one-liner:

.. code-block:: console

   (.venv) $ sudo curl -sSL https://raw.githubusercontent.com/Ballistyxx/blimp-utils/refs/heads/main/install.sh | sh
