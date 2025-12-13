Cal-Poly-ME-405 Romi Robot Project
================================

Overview
--------

This repository contains the code and documentation for a Romi-based mobile
robot used in the ME 405 Mechatronics course. The goal of the project is to
design, implement, and document a complete embedded control system for the
Romi platform, including sensing, estimation, and closed-loop control.

The repository is organized so that all of the embedded code lives in the
``code/`` directory and all written documentation is built with Sphinx and
hosted on Read the Docs.

Hardware
--------

- Pololu Romi chassis and motors
- Nucleo microcontroller board
- Line sensor array
- IMU
- 

Repository layout
-----------------

- ``code/`` – All Python files that run on the Nucleo / Romi
  (motor control, sensing, tasks, etc.)
- ``docs/`` – Sphinx documentation source used to build the online docs
- ``images/`` – Figures and diagrams included in the documentation
- ``lumache.py`` – Example module from the Read the Docs tutorial template
- ``pyproject.toml`` – Python packaging / dependency information
- ``README.rst`` – This file

Getting started
---------------

Clone the repository:

.. code-block:: bash

   git clone https://github.com/<your-username>/mech-romi-files-.git
   cd mech-romi-files-

.. code-block:: bash

   python -m venv .venv
   source .venv/bin/activate    # On Windows: .venv\\Scripts\\activate

Install the Python dependencies:

.. code-block:: bash

   pip install -r docs/requirements.txt

Running the robot code
----------------------

All embedded code for the Romi lives in the ``code/`` directory. Typical files
include:

- ``data_col.py`` – Data collection / logging
- ``left_ME_gen.py`` and ``right_ME_gen.py`` – Motor encoder / control tasks
- ``PI_con.py`` – PI controller implementation
- ``IMU.py`` – IMU interface
- ``track.py`` – High-level trajectory or line-following behavior
- ``task_share.py`` – Task and shared variable utilities

How you run these files depends on your course setup (for example, using
MicroPython or a specific Nucleo toolchain). In general:

1. Copy the relevant files from ``code/`` onto the board (or use your class
   workflow to sync them).
2. Set the desired main script (for example ``boot.py`` or another driver
   script).
3. Reset the board and observe the Romi behavior.

See the full documentation for detailed instructions specific to this project.

Documentation
-------------

The written report and user documentation are built with Sphinx and hosted on
Read the Docs.

Online documentation :

- Read the Docs: `<https://mech-romi-files.readthedocs.io/en/latest/index.html>`_

To build the docs locally:

.. code-block:: bash

   cd docs
   make html

Then open ``_build/html/index.html`` in your browser.

Typical documentation sections include:

- Introduction and project requirements
- Hardware description and wiring diagrams
- Control design and calculations
- Task breakdown and software architecture
- Known issues and future work
- Embedded code listings


Authors
-------

- Jireh Velasquez
- Aubrey Perez


